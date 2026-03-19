#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import time
import math
import subprocess
import os
import signal
import threading
from threading import Lock
from dataclasses import dataclass
from typing import Dict, Optional
import yaml
import cv2
import numpy as np

from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from aruco_markers_msgs.msg import MarkerArray as ArucoMarkerArray
from std_srvs.srv import Trigger, Empty
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import tf2_geometry_msgs  # CRITICAL: This enables PoseStamped transforms
from rclpy.duration import Duration


@dataclass
class CameraConfig:
    topic: str
    frame_id: str
    detection_count: int = 0
    last_detection_time: float = 0.0


@dataclass
class ArUcoMarkerPose:
    id: int
    x: float
    y: float
    z: float
    yaw: float
    camera_source: str
    timestamp: float
    detection_count: int = 1


class ArUcoMapAnnotationNode(Node):
    def __init__(self):
        super().__init__('aruco_map_annotation_node')

        self.service_callback_group = ReentrantCallbackGroup()
        self.data_callback_group = MutuallyExclusiveCallbackGroup()
        self.data_lock = threading.RLock()

        # Core parameters
        self.target_frame = 'map'
        self.robot_frame = 'base_link'
        self.starting_pose_delay = 3.0
        self.auto_save_on_shutdown = True
        self.map_save_directory = './saved_maps'
        self.pose_smoothing_weight = 0.3
        self.transform_timeout = 2.0

        # Logging control parameters
        self.log_detection_updates = False  # Set to False to reduce log spam
        self.log_detection_interval = 10  # Only log every N detections per marker

        # Map annotation parameters
        self.start_pose_color = (0, 255, 0)  # Green for start pose (BGR format for OpenCV)
        self.aruco_marker_color = (0, 0, 255)  # Red for ArUco markers (BGR format)
        self.annotation_radius = 8  # Pixel radius for annotations
        self.text_color = (255, 255, 255)  # White text
        self.text_thickness = 2

        # TF setup - FIX: Pass self (the node) to TransformListener
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_ready = False

        # Data storage
        self.marker_poses: Dict[int, ArUcoMarkerPose] = {}
        self.starting_pose: Optional[PoseStamped] = None
        self.current_robot_pose: Optional[PoseStamped] = None
        self.current_map: Optional[OccupancyGrid] = None
        self.shutdown_in_progress = False

        # Setup camera configurations
        self.setup_camera_configs()

        # Setup subscriptions, publishers, and services
        self.setup_communication()

        # Setup signal handlers for graceful shutdown and auto-save
        self.setup_shutdown_handlers()

        # Timers for various periodic tasks
        if self.starting_pose_delay > 0:
            self.starting_pose_timer = self.create_timer(self.starting_pose_delay, self.capture_starting_pose)

        self.status_timer = self.create_timer(10.0, self.log_status)
        self.pose_update_timer = self.create_timer(0.1, self.update_robot_pose)
        self.tf_check_timer = self.create_timer(5.0, self.check_tf_status)  # Check more frequently
        self.viz_timer = self.create_timer(1.0, self.publish_visualization)

        # Add TF debug timer
        self.tf_debug_timer = self.create_timer(15.0, self.debug_tf_tree)

        self.get_logger().info("🚀 ArUco Map Annotation Node started and ready.")

    def setup_camera_configs(self):
        # TODO: Update these frame names to match your actual camera setup
        # Check with: ros2 run tf2_tools view_frames
        self.camera_configs = {
            'front': CameraConfig('/front_cam/aruco/markers', 'front_cam_optical_frame'),
            'rear': CameraConfig('/back_cam/aruco/markers', 'back_cam_optical_frame'),  # or 'rear_cam_optical_frame'?
            'left': CameraConfig('/left_cam/aruco/markers', 'left_cam_optical_frame'),
            'right': CameraConfig('/right_cam/aruco/markers', 'right_cam_optical_frame'),
        }
        
        # Log the expected frames
        self.get_logger().info("Expected camera frames:")
        for name, cfg in self.camera_configs.items():
            self.get_logger().info(f"  {name}: {cfg.frame_id} (topic: {cfg.topic})")

    def setup_communication(self):
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=5,
        )

        # Subscribe to map topic to get current map for annotation
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            reliable_qos,
            callback_group=self.data_callback_group,
        )

        self.aruco_subscribers = {}
        for name, cfg in self.camera_configs.items():
            self.get_logger().info(f"Subscribing to {cfg.topic} for camera '{name}'")
            self.aruco_subscribers[name] = self.create_subscription(
                ArucoMarkerArray,
                cfg.topic,
                lambda msg, cam=name: self.aruco_callback(msg, cam),
                sensor_qos,
                callback_group=self.data_callback_group,
            )
            self.get_logger().info(f"Subscribed to {cfg.topic} for camera '{name}'")

        self.marker_pub = self.create_publisher(MarkerArray, 'aruco_map_markers', reliable_qos)

        self.save_map_service = self.create_service(
            Trigger,
            'save_annotated_map',
            self.save_map_service_callback,
            callback_group=self.service_callback_group,
        )

        self.capture_pose_service = self.create_service(
            Trigger,
            'capture_start_pose',
            self.capture_start_pose_service_callback,
            callback_group=self.service_callback_group,
        )

        self.rtabmap_save_client = self.create_client(Trigger, '/rtabmap/save_2d_map')
        self.rtabmap_trigger_client = self.create_client(Empty, '/rtabmap/trigger_new_map')
        self.map_server_save_client = self.create_client(Trigger, '/map_server/save_map')

    def map_callback(self, msg: OccupancyGrid):
        """Store the current map for annotation"""
        self.current_map = msg
        self.get_logger().debug(f"Map updated: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}")

    def world_to_map_coords(self, world_x: float, world_y: float) -> tuple:
        """Convert world coordinates to map pixel coordinates"""
        if not self.current_map:
            return None, None
        
        # Map origin is at map.info.origin.position
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        resolution = self.current_map.info.resolution
        
        # Convert world coordinates to map coordinates
        map_x = int((world_x - origin_x) / resolution)
        map_y = int((world_y - origin_y) / resolution)
        
        # Flip Y coordinate because image Y axis is inverted
        map_y = self.current_map.info.height - map_y - 1
        
        # Check bounds
        if (0 <= map_x < self.current_map.info.width and 
            0 <= map_y < self.current_map.info.height):
            return map_x, map_y
        else:
            return None, None

    def annotate_map_image(self, map_image_path: str) -> str:
        """Add colored annotations to the map image"""
        try:
            # Load the map image
            if not os.path.exists(map_image_path):
                self.get_logger().warn(f"Map image not found: {map_image_path}")
                return map_image_path
            
            # Load image in grayscale and convert to BGR for color annotations
            img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                self.get_logger().error(f"Failed to load map image: {map_image_path}")
                return map_image_path
            
            # Convert to BGR for color annotations
            img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            
            self.get_logger().info(f"Annotating map image: {img_color.shape}")
            
            # Annotate starting pose (GREEN)
            if self.starting_pose:
                start_x, start_y = self.world_to_map_coords(
                    self.starting_pose.pose.position.x,
                    self.starting_pose.pose.position.y
                )
                if start_x is not None and start_y is not None:
                    # Draw filled circle for start position
                    cv2.circle(img_color, (start_x, start_y), self.annotation_radius, 
                              self.start_pose_color, -1)  # -1 for filled circle
                    
                    # Draw direction arrow
                    yaw = self.quaternion_to_yaw(self.starting_pose.pose.orientation)
                    arrow_length = self.annotation_radius * 2
                    end_x = int(start_x + arrow_length * math.cos(yaw))
                    end_y = int(start_y - arrow_length * math.sin(yaw))  # Negative because Y is flipped
                    
                    cv2.arrowedLine(img_color, (start_x, start_y), (end_x, end_y), 
                                  self.start_pose_color, 3, tipLength=0.3)
                    
                    # Add "START" label
                    cv2.putText(img_color, "START", (start_x + 15, start_y - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.text_color, self.text_thickness)
                    
                    self.get_logger().info(f"Annotated start pose at ({start_x}, {start_y})")
            
            # Annotate ArUco markers (RED)
            with self.data_lock:
                for marker_id, pose in self.marker_poses.items():
                    marker_x, marker_y = self.world_to_map_coords(pose.x, pose.y)
                    if marker_x is not None and marker_y is not None:
                        # Draw filled circle for marker
                        cv2.circle(img_color, (marker_x, marker_y), self.annotation_radius, 
                                  self.aruco_marker_color, -1)  # -1 for filled circle
                        
                        # Draw orientation arrow
                        arrow_length = self.annotation_radius * 1.5
                        end_x = int(marker_x + arrow_length * math.cos(pose.yaw))
                        end_y = int(marker_y - arrow_length * math.sin(pose.yaw))  # Negative because Y is flipped
                        
                        cv2.arrowedLine(img_color, (marker_x, marker_y), (end_x, end_y), 
                                      self.aruco_marker_color, 2, tipLength=0.3)
                        
                        # Add marker ID label
                        label = f"ID:{marker_id}"
                        cv2.putText(img_color, label, (marker_x + 15, marker_y - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.text_color, self.text_thickness)
                        
                        self.get_logger().info(f"Annotated ArUco marker {marker_id} at ({marker_x}, {marker_y})")
            
            # Save annotated image
            base_name = os.path.splitext(map_image_path)[0]
            annotated_path = f"{base_name}_annotated.png"
            
            success = cv2.imwrite(annotated_path, img_color)
            if success:
                self.get_logger().info(f"Annotated map saved to: {annotated_path}")
                return annotated_path
            else:
                self.get_logger().error(f"Failed to save annotated map to: {annotated_path}")
                return map_image_path
                
        except Exception as e:
            self.get_logger().error(f"Error annotating map image: {e}")
            return map_image_path

    def setup_shutdown_handlers(self):
        signal.signal(signal.SIGINT, self.signal_shutdown)
        signal.signal(signal.SIGTERM, self.signal_shutdown)

    def signal_shutdown(self, signum, frame):
        if not self.shutdown_in_progress:
            self.shutdown_in_progress = True
            self.get_logger().info(f"Received shutdown signal ({signum}). Saving map before exiting...")
            if self.marker_poses:
                success, msg = self.save_map_internal()
                if success:
                    self.get_logger().info(f"Shutdown: Successfully saved map: {msg}")
                else:
                    self.get_logger().error(f"Shutdown: Failed to save map: {msg}")
            else:
                self.get_logger().info("Shutdown: No markers detected, skipping map save")
            self.get_logger().info("Shutting down node...")
            rclpy.shutdown()

    def aruco_callback(self, msg: ArucoMarkerArray, camera_name: str):
        if self.shutdown_in_progress or not msg.markers:
            return

        current_time = time.time()
        cfg = self.camera_configs.get(camera_name)
        if not cfg:
            self.get_logger().warn(f"No camera config for '{camera_name}', skipping message")
            return

        cfg.detection_count += len(msg.markers)
        cfg.last_detection_time = current_time

        self.get_logger().debug(f"Callback from camera '{camera_name}' with {len(msg.markers)} markers")
        with self.data_lock:
            for marker in msg.markers:
                try:
                    marker_id = marker.id
                    map_pose = self.transform_marker_pose(marker, cfg.frame_id, msg.header)
                    if not map_pose:
                        self.get_logger().warn(f"Failed to transform marker {marker_id} from camera '{camera_name}'")
                        continue
                    x = map_pose.pose.position.x
                    y = map_pose.pose.position.y
                    z = map_pose.pose.position.z
                    yaw = self.quaternion_to_yaw(map_pose.pose.orientation)

                    # Only log new marker detections, not every update
                    is_new_marker = marker_id not in self.marker_poses

                    if marker_id in self.marker_poses:
                        old = self.marker_poses[marker_id]
                        w = self.pose_smoothing_weight
                        new_x = w * x + (1 - w) * old.x
                        new_y = w * y + (1 - w) * old.y
                        new_z = w * z + (1 - w) * old.z
                        new_yaw = w * yaw + (1 - w) * old.yaw

                        self.marker_poses[marker_id] = ArUcoMarkerPose(
                            id=marker_id,
                            x=new_x,
                            y=new_y,
                            z=new_z,
                            yaw=new_yaw,
                            camera_source=camera_name,
                            timestamp=current_time,
                            detection_count=old.detection_count + 1,
                        )
                        
                        # Only log every N-th detection to reduce spam
                        if (self.log_detection_updates and 
                            (old.detection_count + 1) % self.log_detection_interval == 0):
                            self.get_logger().info(f"Updated marker {marker_id} (detections: {old.detection_count + 1})")
                    else:
                        self.marker_poses[marker_id] = ArUcoMarkerPose(
                            id=marker_id,
                            x=x,
                            y=y,
                            z=z,
                            yaw=yaw,
                            camera_source=camera_name,
                            timestamp=current_time,
                        )
                        self.get_logger().info(f"NEW marker {marker_id} detected at ({x:.2f}, {y:.2f}, {z:.2f}) yaw {math.degrees(yaw):.1f}° from camera '{camera_name}'")
                        
                except Exception as e:
                    self.get_logger().error(f"Error processing marker from camera '{camera_name}': {str(e)}")

    def transform_marker_pose(self, marker, source_frame, header) -> Optional[PoseStamped]:
        try:
            pose_stamped = PoseStamped()
            # FIX: Handle different marker message structures more robustly
            if hasattr(marker, "pose") and hasattr(marker.pose, "header") and hasattr(marker.pose, "pose"):
                pose_stamped.header = marker.pose.header
                pose_stamped.pose = marker.pose.pose
            else:
                pose_stamped.header.frame_id = source_frame
                pose_stamped.header.stamp = header.stamp
                pose_stamped.pose = marker.pose

            # Check if source frame exists
            try:
                self.tf_buffer.lookup_transform(
                    self.target_frame, 
                    source_frame, 
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5)
                )
            except Exception as tf_check_e:
                self.get_logger().warn(f"TF chain not available from {source_frame} to {self.target_frame}: {tf_check_e}")
                return None

            # Try using tf2_geometry_msgs transform first
            try:
                transformed = self.tf_buffer.transform(
                    pose_stamped, 
                    self.target_frame, 
                    timeout=Duration(seconds=self.transform_timeout)
                )
                return transformed
            except Exception as tf2_error:
                self.get_logger().debug(f"tf2_geometry_msgs transform failed, trying manual transform: {tf2_error}")
                
                # Fallback: Manual transformation using transform lookup
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.target_frame,
                        pose_stamped.header.frame_id,
                        pose_stamped.header.stamp if pose_stamped.header.stamp.sec != 0 else rclpy.time.Time(),
                        timeout=Duration(seconds=self.transform_timeout)
                    )
                    
                    # Manual pose transformation (simplified)
                    transformed_pose = PoseStamped()
                    transformed_pose.header.frame_id = self.target_frame
                    transformed_pose.header.stamp = pose_stamped.header.stamp
                    
                    # Apply translation
                    transformed_pose.pose.position.x = (
                        pose_stamped.pose.position.x + transform.transform.translation.x
                    )
                    transformed_pose.pose.position.y = (
                        pose_stamped.pose.position.y + transform.transform.translation.y
                    )
                    transformed_pose.pose.position.z = (
                        pose_stamped.pose.position.z + transform.transform.translation.z
                    )
                    
                    # For now, just copy the orientation (this is simplified)
                    # In a full implementation, you'd properly compose the quaternions
                    transformed_pose.pose.orientation = pose_stamped.pose.orientation
                    
                    return transformed_pose
                    
                except Exception as manual_error:
                    self.get_logger().warn(f"Manual transform also failed: {manual_error}")
                    return None
                    
        except Exception as e:
            self.get_logger().warn(f"TF transform failed for marker {getattr(marker, 'id', '?')} from {source_frame}: {e}")
            return None

    def quaternion_to_yaw(self, q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.robot_frame, 
                rclpy.time.Time(), 
                timeout=Duration(seconds=0.1)
            )
            pose = PoseStamped()
            pose.header = t.header
            pose.pose.position.x = t.transform.translation.x
            pose.pose.position.y = t.transform.translation.y
            pose.pose.position.z = t.transform.translation.z
            pose.pose.orientation = t.transform.rotation
            self.current_robot_pose = pose
            self.get_logger().debug(f"Robot pose updated: {pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}")
        except Exception:
            pass  # Quietly ignore TF exceptions

    def check_tf_status(self):
        try:
            self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
            if not self.tf_ready:
                self.get_logger().info("TF is ready")
            self.tf_ready = True
            
            # Also check camera frame availability
            missing_cameras = []
            for cam_name, cfg in self.camera_configs.items():
                try:
                    self.tf_buffer.lookup_transform(self.target_frame, cfg.frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1))
                except Exception:
                    missing_cameras.append(f"{cam_name}({cfg.frame_id})")
            
            if missing_cameras:
                self.get_logger().warn(f"Camera frames not available in TF: {', '.join(missing_cameras)}")
            
            if hasattr(self, 'tf_check_timer') and not missing_cameras:
                self.tf_check_timer.destroy()
        except Exception as e:
            self.tf_ready = False
            self.get_logger().warn(f"TF not ready: robot frame {self.robot_frame} -> {self.target_frame}: {e}")

    def capture_starting_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.robot_frame, 
                rclpy.time.Time(), 
                timeout=Duration(seconds=self.transform_timeout)
            )
            p = PoseStamped()
            p.header = t.header
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.position.z = t.transform.translation.z
            p.pose.orientation = t.transform.rotation
            self.starting_pose = p
            self.get_logger().info(f"Starting pose captured at ({p.pose.position.x:.2f},{p.pose.position.y:.2f})")
            if hasattr(self, "starting_pose_timer"):
                self.starting_pose_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"Failed to capture starting pose: {e}")

    def capture_start_pose_service_callback(self, request, response):
        try:
            self.capture_starting_pose()
            response.success = True
            response.message = "Starting pose captured successfully."
        except Exception as e:
            response.success = False
            response.message = f"Failed to capture starting pose: {e}"
        return response

    def log_status(self):
        if self.shutdown_in_progress:
            return
        with self.data_lock:
            if self.marker_poses:
                keys = list(self.marker_poses.keys())
                total = sum(p.detection_count for p in self.marker_poses.values())
                self.get_logger().info(f"📍 {len(self.marker_poses)} markers tracked | IDs: {keys} | Total detections: {total}")
                
                # Show detection stats per camera (more concise)
                active_cameras = []
                for cam, cfg in self.camera_configs.items():
                    if cfg.detection_count > 0:
                        ago = time.time() - cfg.last_detection_time
                        active_cameras.append(f"{cam}({cfg.detection_count}, {ago:.1f}s ago)")
                
                if active_cameras:
                    self.get_logger().info(f"📷 Active cameras: {', '.join(active_cameras)}")
            else:
                self.get_logger().info("⏳ No markers detected yet.")

    def publish_visualization(self):
        if self.shutdown_in_progress:
            return
        now = self.get_clock().now()
        ma = MarkerArray()

        if self.starting_pose:
            arrow = self.make_arrow_marker(0, self.starting_pose)
            ma.markers.append(arrow)
            text_marker = self.make_text_marker(10000, self.starting_pose, "START")
            ma.markers.append(text_marker)

        with self.data_lock:
            idx = 1
            for p in self.marker_poses.values():
                marker = Marker()
                marker.header.frame_id = self.target_frame
                marker.header.stamp = now.to_msg()
                marker.ns = "aruco_markers"
                marker.id = idx
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = p.x
                marker.pose.position.y = p.y
                marker.pose.position.z = p.z + 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.4
                marker.scale.y = 0.4
                marker.scale.z = 0.4
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 0.8
                ma.markers.append(marker)

                label = self.make_text_marker(10000 + idx, None, f"ID: {p.id}")
                label.pose.position.x = p.x
                label.pose.position.y = p.y
                label.pose.position.z = p.z + 0.6
                ma.markers.append(label)

                idx += 1

        self.marker_pub.publish(ma)

    def make_arrow_marker(self, id_, pose_stamped):
        m = Marker()
        m.header = pose_stamped.header
        m.id = id_
        m.ns = "start_arrow"
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose_stamped.pose
        m.scale.x = 1.2
        m.scale.y = 0.4
        m.scale.z = 0.4
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.9
        return m

    def make_text_marker(self, id_, p_pose, text):
        m = Marker()
        m.header.frame_id = self.target_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.id = id_
        m.ns = "marker_text"
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.text = text
        m.scale.z = 0.4
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.pose.orientation.w = 1.0
        if p_pose:
            m.pose.position = p_pose.pose.position
            m.pose.position.z += 0.8
        return m

    def save_map_internal(self):
        self.get_logger().info(f"💾 Saving map with {len(self.marker_poses)} markers...")
        
        try:
            # First, save the regular map
            saved_successfully = False
            map_filename = None
            
            # Try RTABMap first
            if self.rtabmap_save_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("Trying RTABMap save service...")
                req = Trigger.Request()
                future = self.rtabmap_save_client.call_async(req)
                
                # Wait for the service call to complete
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result() and future.result().success:
                    self.get_logger().info("RTABMap save successful")
                    saved_successfully = True
                else:
                    self.get_logger().warn(f"RTABMap save failed: {future.result().message if future.result() else 'No response'}")
            
            # Try map_server as backup
            if not saved_successfully and self.map_server_save_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info("Trying map_server save service...")
                req = Trigger.Request()
                future = self.map_server_save_client.call_async(req)
                
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result() and future.result().success:
                    self.get_logger().info("map_server save successful")
                    saved_successfully = True
                else:
                    self.get_logger().warn(f"map_server save failed: {future.result().message if future.result() else 'No response'}")
            
            # Fallback: CLI method with improved PNG handling
            if not saved_successfully:
                self.get_logger().info("Fallback to CLI map saver")
                os.makedirs(self.map_save_directory, exist_ok=True)
                map_filename = f"{self.map_save_directory}/aruco_map_{time.strftime('%Y%m%d-%H%M%S')}"
                
                # Use map_saver_cli with explicit format to ensure PNG creation
                cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_filename, "--fmt", "png"]
                try:
                    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
                    if result.returncode == 0:
                        self.get_logger().info(f"Map saved to {map_filename}")
                        saved_successfully = True
                    else:
                        self.get_logger().error(f"CLI saving failed: {result.stderr}")
                        # Try without --fmt flag as backup
                        cmd_backup = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_filename]
                        result_backup = subprocess.run(cmd_backup, capture_output=True, text=True, timeout=30)
                        if result_backup.returncode == 0:
                            self.get_logger().info(f"Map saved with backup command to {map_filename}")
                            saved_successfully = True
                        else:
                            return False, f"Both CLI saving attempts failed: {result.stderr} | {result_backup.stderr}"
                except subprocess.TimeoutExpired:
                    return False, "CLI map saver timed out"
                except Exception as e:
                    return False, f"CLI saving exception: {e}"
            
            # Now handle annotation - look for the PNG file more systematically
            if saved_successfully:
                png_path = None
                
                if map_filename:
                    # We know the filename, check for PNG file
                    potential_paths = [
                        f"{map_filename}.png",
                        f"{map_filename}.pgm",  # Sometimes saves as PGM
                    ]
                    
                    for path in potential_paths:
                        if os.path.exists(path):
                            png_path = path
                            break
                    
                    # If we found PGM but need PNG, convert it
                    if png_path and png_path.endswith('.pgm'):
                        try:
                            import cv2
                            img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
                            png_converted_path = png_path.replace('.pgm', '.png')
                            cv2.imwrite(png_converted_path, img)
                            png_path = png_converted_path
                            self.get_logger().info(f"Converted PGM to PNG: {png_path}")
                        except Exception as e:
                            self.get_logger().warn(f"Failed to convert PGM to PNG: {e}")
                
                # If we still don't have a PNG, try to find the most recent one
                if not png_path:
                    self.get_logger().info("Searching for most recent map file...")
                    png_path = self.find_most_recent_map_file()
                
                # Annotate the map if we found a PNG file
                if png_path and os.path.exists(png_path):
                    annotated_path = self.annotate_map_image(png_path)
                    self.get_logger().info(f"✅ Map annotated and saved to: {annotated_path}")
                else:
                    self.get_logger().warn("⚠️  Could not find PNG file to annotate")
                
                # Always save marker annotations to a separate file
                if map_filename:
                    self.save_marker_annotations(map_filename)
                else:
                    # Save to timestamped file
                    timestamp = time.strftime('%Y%m%d-%H%M%S')
                    self.save_marker_annotations(f"{self.map_save_directory}/annotations_{timestamp}")
                
                return True, "Map saved and annotated successfully"
            else:
                return False, "Failed to save map through all available methods"
                
        except Exception as e:
            message = f"Exception during map save: {e}"
            self.get_logger().error(message)
            return False, message

    def find_most_recent_map_file(self) -> Optional[str]:
        """Find the most recently created .png or .pgm map file"""
        try:
            # Common map save locations
            search_paths = [
                ".",
                "./maps",
                "./saved_maps", 
                self.map_save_directory,
                os.path.expanduser("~"),
                "/tmp"
            ]
            
            most_recent_file = None
            most_recent_time = 0
            
            for search_path in search_paths:
                if not os.path.exists(search_path):
                    continue
                    
                for file in os.listdir(search_path):
                    # Look for PNG or PGM files that might be maps
                    if (file.endswith(('.png', '.pgm')) and 
                        ('map' in file.lower() or 'rtabmap' in file.lower() or 'aruco' in file.lower())):
                        file_path = os.path.join(search_path, file)
                        file_time = os.path.getmtime(file_path)
                        
                        # Only consider files created in the last 2 minutes
                        if time.time() - file_time < 120 and file_time > most_recent_time:
                            most_recent_time = file_time
                            most_recent_file = file_path
            
            if most_recent_file:
                self.get_logger().info(f"Found recent map file: {most_recent_file}")
                
                # If it's a PGM file, try to convert to PNG for annotation
                if most_recent_file.endswith('.pgm'):
                    try:
                        img = cv2.imread(most_recent_file, cv2.IMREAD_GRAYSCALE)
                        png_path = most_recent_file.replace('.pgm', '_converted.png')
                        cv2.imwrite(png_path, img)
                        self.get_logger().info(f"Converted PGM to PNG: {png_path}")
                        return png_path
                    except Exception as e:
                        self.get_logger().warn(f"Failed to convert PGM: {e}")
                        return most_recent_file
                
                return most_recent_file
            else:
                self.get_logger().warn("No recent map files found")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error searching for map files: {e}")
            return None

    def save_marker_annotations(self, base_filename):
        """Save marker positions and starting pose to a YAML file"""
        try:
            annotations_file = f"{base_filename}_annotations.yaml"
            with open(annotations_file, 'w') as f:
                f.write("# ArUco Map Annotations\n")
                f.write(f"# Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("# Color Legend:\n")
                f.write("#   Start Pose: GREEN circle with direction arrow\n")
                f.write("#   ArUco Markers: RED circles with orientation arrows\n\n")
                
                if self.starting_pose:
                    f.write("starting_pose:\n")
                    f.write(f"  x: {self.starting_pose.pose.position.x:.3f}\n")
                    f.write(f"  y: {self.starting_pose.pose.position.y:.3f}\n")
                    f.write(f"  z: {self.starting_pose.pose.position.z:.3f}\n")
                    yaw = self.quaternion_to_yaw(self.starting_pose.pose.orientation)
                    f.write(f"  yaw: {yaw:.3f}\n")
                    f.write("  annotation_color: \"GREEN\"\n\n")
                
                f.write("aruco_markers:\n")
                for marker_id, pose in self.marker_poses.items():
                    f.write(f"  marker_{marker_id}:\n")
                    f.write(f"    id: {marker_id}\n")
                    f.write(f"    x: {pose.x:.3f}\n")
                    f.write(f"    y: {pose.y:.3f}\n")
                    f.write(f"    z: {pose.z:.3f}\n")
                    f.write(f"    yaw: {pose.yaw:.3f}\n")
                    f.write(f"    camera_source: \"{pose.camera_source}\"\n")
                    f.write(f"    detection_count: {pose.detection_count}\n")
                    f.write(f"    last_seen: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(pose.timestamp))}\n")
                    f.write("    annotation_color: \"RED\"\n")
                
                # Add annotation parameters used
                f.write("\nannotation_settings:\n")
                f.write(f"  start_pose_color_bgr: {list(self.start_pose_color)}\n")
                f.write(f"  aruco_marker_color_bgr: {list(self.aruco_marker_color)}\n")
                f.write(f"  annotation_radius: {self.annotation_radius}\n")
                f.write(f"  text_color_bgr: {list(self.text_color)}\n")
            
            self.get_logger().info(f"📄 Marker annotations saved to {annotations_file}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save marker annotations: {e}")

    def debug_tf_tree(self):
        """Debug method to check TF tree status"""
        self.get_logger().info("=== TF Tree Debug ===")
        
        # Check robot frame
        try:
            self.tf_buffer.lookup_transform(self.target_frame, self.robot_frame, rclpy.time.Time())
            self.get_logger().info(f"✓ {self.robot_frame} -> {self.target_frame} available")
        except Exception as e:
            self.get_logger().warn(f"✗ {self.robot_frame} -> {self.target_frame}: {e}")
        
        # Check each camera frame
        for cam_name, cfg in self.camera_configs.items():
            try:
                self.tf_buffer.lookup_transform(self.target_frame, cfg.frame_id, rclpy.time.Time())
                self.get_logger().info(f"✓ {cfg.frame_id} -> {self.target_frame} available")
            except Exception as e:
                self.get_logger().warn(f"✗ {cfg.frame_id} -> {self.target_frame}: {e}")

    def save_map_service_callback(self, request, response):
        self.get_logger().info("💾 Save map service called")
        success, message = self.save_map_internal()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(f"✅ Map saved successfully: {message}")
        else:
            self.get_logger().error(f"❌ Failed to save map: {message}")
        return response

    def destroy_node(self):
        if not self.shutdown_in_progress:
            self.shutdown_in_progress = True
            if self.auto_save_on_shutdown and self.marker_poses:
                self.get_logger().info("Auto-saving map before node destruction...")
                success, msg = self.save_map_internal()
                if success:
                    self.get_logger().info(f"Auto-save successful: {msg}")
                else:
                    self.get_logger().error(f"Auto-save failed: {msg}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoMapAnnotationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down...")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()