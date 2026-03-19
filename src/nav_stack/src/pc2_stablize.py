#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2, Imu, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2

from message_filters import ApproximateTimeSynchronizer, Subscriber
import struct

class SimpleGroundStabilizer(Node):
    def __init__(self):
        super().__init__('simple_ground_stabilizer')
        
        # Parameters - YOUR EXACT CODE + minimal additions for fixes
        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_topic', '/panther/imu/data'),
                ('pointcloud_topic', '/panther/cx/lslidar_point_cloud'),
                ('odometry_topic', '/panther/odometry/filtered'),
                ('output_topic', '/ground_stabilized_points'),
                ('target_frame', 'panther/base_link'),
                ('debug_mode', True),
                ('invert_pitch', False),
                ('invert_roll', False),
                ('invert_yaw', True),
                ('stabilization_strength', 1.0),
                ('yaw_strength', 0.3),
                ('use_odom_yaw', True),
                # MINIMAL ADDITIONS for the two specific problems:
                ('apply_height_fix', True),        # Fix for ground plane height issue
                ('height_offset', 0.32999),            # Manual height adjustment if needed
                ('stabilized_frame', 'panther/base_link'),    # Keep in base_link but fix spin handling
            ]
        )
        
        # Get parameters - YOUR EXACT CODE + minimal additions
        self.imu_topic = self.get_parameter('imu_topic').value
        self.pc_topic = self.get_parameter('pointcloud_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.debug_mode = self.get_parameter('debug_mode').value
        self.invert_pitch = self.get_parameter('invert_pitch').value
        self.invert_roll = self.get_parameter('invert_roll').value
        self.invert_yaw = self.get_parameter('invert_yaw').value
        self.strength = self.get_parameter('stabilization_strength').value
        self.yaw_strength = self.get_parameter('yaw_strength').value
        self.use_odom_yaw = self.get_parameter('use_odom_yaw').value
        # New parameters
        self.apply_height_fix = self.get_parameter('apply_height_fix').value
        self.height_offset = self.get_parameter('height_offset').value
        self.stabilized_frame = self.get_parameter('stabilized_frame').value
        
        # QoS - YOUR EXACT CODE
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Synchronizer - YOUR EXACT CODE
        self.imu_sub = Subscriber(self, Imu, self.imu_topic, qos_profile=sensor_qos)
        self.pc_sub = Subscriber(self, PointCloud2, self.pc_topic, qos_profile=sensor_qos)
        
        if self.use_odom_yaw:
            self.odom_sub = Subscriber(self, Odometry, self.odometry_topic, qos_profile=sensor_qos)
            self.sync = ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub, self.odom_sub], queue_size=10, slop=0.1)
            self.sync.registerCallback(self.sync_callback_with_odom)
        else:
            self.sync = ApproximateTimeSynchronizer([self.pc_sub, self.imu_sub], queue_size=10, slop=0.1)
            self.sync.registerCallback(self.sync_callback)
        
        # Publisher - YOUR EXACT CODE
        self.stabilized_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        # Reference orientation - YOUR EXACT CODE + minimal additions
        self.reference_orientation = None
        self.reference_yaw = None
        self.processed_clouds = 0
        # MINIMAL ADDITION for height fix:
        self.reference_ground_z = None  # Store initial ground reference
        
        self.create_timer(5.0, self.status_callback)
        
        self.get_logger().info("=== Simple Ground Stabilizer (Fixed) ===")
        self.get_logger().info(f"Height fix enabled: {self.apply_height_fix}")
        self.get_logger().info(f"Manual height offset: {self.height_offset}")
        self.get_logger().info(f"Stabilized frame: {self.stabilized_frame}")
        self.get_logger().info(f"Use odometry yaw: {self.use_odom_yaw}")
        self.get_logger().info(f"Invert Pitch: {self.invert_pitch}")
        self.get_logger().info(f"Invert Roll: {self.invert_roll}")
        self.get_logger().info(f"Invert Yaw: {self.invert_yaw}")
        self.get_logger().info(f"Stabilization Strength: {self.strength}")
        
    def status_callback(self):
        self.get_logger().info(f"Processed {self.processed_clouds} point clouds")
    
    def get_roll_pitch_from_imu(self, imu_msg):
        """Extract roll and pitch angles from IMU - YOUR EXACT CODE"""
        try:
            q = imu_msg.orientation
            quat = [q.x, q.y, q.z, q.w]  # scipy format
            
            # Normalize quaternion
            norm = np.linalg.norm(quat)
            if norm < 1e-6:
                return 0.0, 0.0
            quat = [q/norm for q in quat]
            
            # Convert to rotation and extract roll, pitch
            rotation = R.from_quat(quat)
            euler = rotation.as_euler('xyz')  # Roll, Pitch, Yaw
            
            roll = euler[0]   # Rotation around X-axis (left/right tilt)
            pitch = euler[1]  # Rotation around Y-axis (forward/backward tilt)
            
            return roll, pitch
            
        except Exception as e:
            self.get_logger().error(f"Error extracting roll/pitch: {str(e)}")
            return 0.0, 0.0
    
    def get_yaw_from_odometry(self, odom_msg):
        """Extract yaw from odometry - YOUR EXACT CODE"""
        try:
            q = odom_msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            
            # Normalize quaternion
            norm = np.linalg.norm(quat)
            if norm < 1e-6:
                return 0.0
            quat = [q/norm for q in quat]
            
            rotation = R.from_quat(quat)
            euler = rotation.as_euler('xyz')
            
            yaw = euler[2]  # Rotation around Z-axis (heading)
            return yaw
            
        except Exception as e:
            self.get_logger().error(f"Error extracting yaw: {str(e)}")
            return 0.0
    
    # MINIMAL ADDITION - Simple ground height estimation
    def get_ground_reference_z(self, points):
        """Get a simple ground height reference"""
        try:
            # Simple approach: use 10th percentile of Z values as ground estimate
            z_values = points[:, 2]
            ground_z = np.percentile(z_values, 10)
            return ground_z
        except:
            return 0.0
    
    def create_stabilization_matrix(self, roll, pitch, yaw=0.0):
        """Create transformation matrix - YOUR EXACT CODE"""
        try:
            # Apply inversion flags if coordinate frames are mismatched
            if self.invert_roll:
                roll = -roll
            if self.invert_pitch:
                pitch = -pitch
            if self.invert_yaw:
                yaw = -yaw
            
            # Scale by stabilization strength (allows gradual tuning)
            roll *= self.strength
            pitch *= self.strength
            yaw *= self.yaw_strength
            
            # Create rotation matrices to UNDO the tilt - YOUR EXACT METHOD
            roll_correction = R.from_euler('x', -roll)    # Undo roll
            pitch_correction = R.from_euler('y', -pitch)  # Undo pitch
            yaw_correction = R.from_euler('z', -yaw)      # Undo yaw
            
            # Combine corrections - YOUR EXACT METHOD
            stabilization_rotation = yaw_correction * pitch_correction * roll_correction
            
            return stabilization_rotation.as_matrix()
            
        except Exception as e:
            self.get_logger().error(f"Error creating stabilization matrix: {str(e)}")
            return np.eye(3)
    
    def extract_points(self, pc_msg):
        """Extract points from point cloud - YOUR EXACT CODE (with your typo fix)"""
        try:
            points_gen = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
            points_list = list(points_gen)
            
            if len(points_list) < 10:
                return None, None
            
            # Try to get intensity
            try:
                intensity_gen = pc2.read_points(pc_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)
                intensity_list = list(intensity_gen)
                
                points_array = [[p[0], p[1], p[2]] for p in intensity_list]
                intensities_array = [p[3] for p in intensity_list]
                
            except:
                # FIXED: your correction
                points_array = [[p[0], p[1], p[2]] for p in points_list]
                intensities_array = [0.0 for _ in points_list]
            
            points = np.array(points_array, dtype=np.float32)
            intensities = np.array(intensities_array, dtype=np.float32)
            
            # Filter NaN/Inf
            valid_mask = np.all(np.isfinite(points), axis=1)
            points = points[valid_mask]
            intensities = intensities[valid_mask]
            
            return points, intensities
            
        except Exception as e:
            self.get_logger().error(f"Point extraction error: {str(e)}")
            return None, None
    
    def sync_callback(self, pc_msg, imu_msg):
        """Original callback - YOUR EXACT CODE"""
        try:
            # Get current robot tilt
            roll, pitch = self.get_roll_pitch_from_imu(imu_msg)
            
            if self.debug_mode and self.processed_clouds % 30 == 0:
                self.get_logger().info(f"Robot tilt - Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°")
            
            # Create stabilization transformation
            stabilization_matrix = self.create_stabilization_matrix(roll, pitch)
            
            # Extract points
            points, intensities = self.extract_points(pc_msg)
            if points is None:
                return
            
            # Apply stabilization - transform points to stay level
            stabilized_points = (stabilization_matrix @ points.T).T
            
            # Create output message
            stabilized_pc = self.create_output_pointcloud(
                stabilized_points, intensities, pc_msg.header.stamp
            )
            
            if stabilized_pc is not None:
                self.stabilized_pub.publish(stabilized_pc)
                self.processed_clouds += 1
                
        except Exception as e:
            self.get_logger().error(f"Stabilization error: {str(e)}")
    
    def sync_callback_with_odom(self, pc_msg, imu_msg, odom_msg):
        """Enhanced callback with odometry yaw - YOUR CODE + SPIN FIX"""
        try:
            # Get current robot tilt from IMU (YOUR METHOD)
            roll, pitch = self.get_roll_pitch_from_imu(imu_msg)
            
            # Get yaw from odometry (YOUR METHOD)
            yaw = self.get_yaw_from_odometry(odom_msg)
            
            # Extract points first (needed for ground reference)
            points, intensities = self.extract_points(pc_msg)
            if points is None:
                return
            
            # Set reference from first reading (YOUR EXACT METHOD + ground reference)
            if self.reference_orientation is None:
                self.reference_orientation = (roll, pitch)
                self.reference_yaw = yaw
                # MINIMAL ADDITION: Store ground reference for height fix
                if self.apply_height_fix:
                    self.reference_ground_z = self.get_ground_reference_z(points)
                self.get_logger().info(f"Reference set - Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°, Yaw: {np.degrees(yaw):.1f}°")
                if self.apply_height_fix:
                    self.get_logger().info(f"Ground reference Z: {self.reference_ground_z:.3f}m")
                return
            
            # Calculate relative angles from reference (YOUR EXACT METHOD)
            ref_roll, ref_pitch = self.reference_orientation
            relative_roll = roll - ref_roll
            relative_pitch = pitch - ref_pitch
            relative_yaw = yaw - self.reference_yaw
            
            # Handle yaw wraparound (YOUR EXACT METHOD)
            if relative_yaw > np.pi:
                relative_yaw -= 2 * np.pi
            elif relative_yaw < -np.pi:
                relative_yaw += 2 * np.pi
            
            if self.debug_mode and self.processed_clouds % 30 == 0:
                self.get_logger().info(f"Relative tilt - Roll: {np.degrees(relative_roll):.1f}°, Pitch: {np.degrees(relative_pitch):.1f}°, Yaw: {np.degrees(relative_yaw):.1f}°")
            
            # REAL SPIN FIX: The problem is we need to stabilize relative to initial orientation
            # but NOT relative to current yaw during spinning - this causes the misalignment
            
            # Create stabilization matrix - but handle yaw differently during spin
            if abs(relative_yaw) > 0.1:  # Robot is spinning significantly  
                # During spin: Only stabilize roll/pitch, let yaw follow robot naturally
                # This keeps the stabilized cloud properly aligned with robot coordinate system
                stabilization_matrix = self.create_stabilization_matrix(relative_roll, relative_pitch, 0.0)
                
                if self.debug_mode and self.processed_clouds % 30 == 0:
                    self.get_logger().info(f"Spinning detected: Using roll/pitch only stabilization")
            else:
                # Normal operation: full stabilization including yaw
                stabilization_matrix = self.create_stabilization_matrix(relative_roll, relative_pitch, relative_yaw)
            
            stabilized_points = (stabilization_matrix @ points.T).T
            
            # MINIMAL HEIGHT FIX - Apply simple ground height correction
            if self.apply_height_fix and self.reference_ground_z is not None:
                current_ground_z = self.get_ground_reference_z(stabilized_points)
                height_correction = self.reference_ground_z - current_ground_z + self.height_offset
                stabilized_points[:, 2] += height_correction
                
                if self.debug_mode and self.processed_clouds % 30 == 0:
                    self.get_logger().info(f"Height correction applied: {height_correction:.3f}m")
            
            # Create output message with correct frame
            stabilized_pc = self.create_output_pointcloud(
                stabilized_points, intensities, pc_msg.header.stamp
            )
            
            if stabilized_pc is not None:
                self.stabilized_pub.publish(stabilized_pc)
                self.processed_clouds += 1
                
        except Exception as e:
            self.get_logger().error(f"Stabilization error: {str(e)}")
    
    def create_output_pointcloud(self, points, intensities, timestamp):
        """Create output point cloud message - YOUR EXACT CODE"""
        try:
            header = Header()
            header.stamp = timestamp
            header.frame_id = self.stabilized_frame  # Use stabilized frame instead of target_frame
            
            cloud_data = bytearray()
            points_written = 0
            
            for i in range(len(points)):
                x, y, z = points[i]
                intensity = intensities[i]
                
                if all(np.isfinite([x, y, z, intensity])):
                    cloud_data.extend(struct.pack('ffff', float(x), float(y), float(z), float(intensity)))
                    points_written += 1
            
            if points_written == 0:
                return None
            
            pc_msg = PointCloud2()
            pc_msg.header = header
            pc_msg.height = 1
            pc_msg.width = points_written
            pc_msg.is_dense = True
            pc_msg.is_bigendian = False
            
            pc_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            
            pc_msg.point_step = 16
            pc_msg.row_step = pc_msg.point_step * pc_msg.width
            pc_msg.data = bytes(cloud_data)
            
            return pc_msg
            
        except Exception as e:
            self.get_logger().error(f"Output creation error: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    
    try:
        stabilizer = SimpleGroundStabilizer()
        print("Starting simple ground stabilizer (fixed)...")
        rclpy.spin(stabilizer)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()