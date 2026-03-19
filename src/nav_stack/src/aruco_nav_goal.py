#!/usr/bin/env python3
"""
Complete Multi-Camera Marker Navigation Node with Side-Aware Safe Pose Logic
Integrates full navigation system with left/right offset strategies based on start pose reference
FIXED: Cross-product side determination (Map saving and marker pose saving removed)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped, Point, Pose, PoseWithCovarianceStamped
from aruco_markers_msgs.msg import MarkerArray
from std_srvs.srv import SetBool, Trigger, Empty
from std_msgs.msg import Int32, Bool
from nav_msgs.srv import GetMap
import tf2_ros
import tf2_geometry_msgs
from dataclasses import dataclass
from typing import Dict, Set, Optional, List
import math
import time
import threading
from enum import Enum
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration

# ==================== Enhanced Data Structures ====================

@dataclass
class CameraConfig:
    topic: str
    frame_id: str
    detection_count: int = 0
    last_detection_time: float = 0.0

@dataclass 
class MarkerPose:
    x: float
    y: float
    z: float
    yaw: float
    timestamp: float
    camera_source: str
    confidence: float = 1.0
    detection_count: int = 1

@dataclass
class SafePose:
    """Enhanced data structure for safe goal pose calculations with side awareness"""
    original_x: float
    original_y: float
    safe_x: float
    safe_y: float
    safe_yaw: float
    offset_distance: float
    robot_x: float
    robot_y: float
    # Side determination fields
    side: str  # 'left' or 'right' relative to start pose reference line
    cross_product: float  # Value used for side determination
    offset_method: str  # 'left_side' or 'right_side' method used
    start_reference_used: bool = True  # Whether start pose was available for calculation

@dataclass
class StartPoseReference:
    """Reference line defined by robot's starting pose and orientation"""
    x0: float  # Starting position x
    y0: float  # Starting position y
    theta0: float  # Starting orientation (yaw)
    cos_theta0: float  # Precomputed cosine for efficiency
    sin_theta0: float  # Precomputed sine for efficiency

class NavigationState(Enum):
    INITIALIZING = 1
    WAITING_FOR_MARKERS = 2
    NAVIGATING_TO_TARGET = 3
    MISSION_COMPLETE = 4
    RETURNING_HOME = 5
    FAILED = 6

# ==================== Main Navigation Node ====================

class MultiCameraMarkerNavigation(Node):
    def __init__(self):
        super().__init__('multi_camera_marker_navigation')
        
        # Initialize parameters
        self.declare_and_load_parameters()
        
        # Threading setup
        self.service_callback_group = ReentrantCallbackGroup()
        self.navigation_callback_group = MutuallyExclusiveCallbackGroup()
        self.data_lock = threading.RLock()
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer(node=self, cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_ready = False
        
        # Navigation setup
        self.nav_action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', 
            callback_group=self.navigation_callback_group
        )
        self.current_nav_goal_handle = None
        self.navigation_start_time = None
        self.navigation_in_progress = False
        
        # Data storage
        self.setup_camera_configs()
        self.marker_poses: Dict[int, MarkerPose] = {}
        self.starting_pose: Optional[PoseStamped] = None
        self.starting_pose_saved = False
        self.current_robot_pose: Optional[PoseStamped] = None
        
        # Enhanced safe pose tracking with side awareness
        self.current_safe_pose: Optional[SafePose] = None
        self.start_reference: Optional[StartPoseReference] = None
        
        # Navigation state
        self.state = NavigationState.INITIALIZING
        self.current_target_id: Optional[int] = None
        self.visited_markers: Set[int] = set()
        self.navigation_sequence: List[int] = []
        self.current_sequence_index = 0
        self.navigation_attempts = {}
        self.required_target_marker = 0
        
        # Proximity tracking
        self.last_proximity_check_time = 0.0
        self.proximity_check_interval = 1.0
        self.last_robot_position = None
        
        # Logging control
        self.last_state_log = None
        self.last_status_log_time = 0
        self.last_proximity_log_time = 0
        self.proximity_log_interval = 5.0
        self.last_diagnostic_log_time = 0
        self.diagnostic_log_interval = 30.0
        
        # Setup publishers, subscribers, services
        self.setup_communication()
        
        # Timers
        self.setup_timers()
            
        self.get_logger().info("🚀 Enhanced Multi-Camera Marker Navigation System with Side-Aware Safe Poses ready!")
        self.get_logger().info(f"📏 Proximity radius set to: {self.proximity_radius}m")
        self.get_logger().info(f"🛡️ Left side offset: {self.left_offset_distance}m, Right side offset: {self.right_offset_distance}m")
        self.get_logger().info(f"🔄 Left strategy: {self.left_offset_strategy}, Right strategy: {self.right_offset_strategy}")
        
    def declare_and_load_parameters(self):
        """Enhanced parameter declaration with dual offset system"""
        # Basic parameters
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('robot_frame', 'panther/base_link')
        self.declare_parameter('fallback_frame', 'panther/base_link')
        self.declare_parameter('transform_timeout', 2.0)
        self.declare_parameter('auto_save_starting_pose', True)
        self.declare_parameter('starting_pose_delay', 3.0)
        self.declare_parameter('navigation_timeout', 60.0)
        self.declare_parameter('max_navigation_attempts', 3)
        self.declare_parameter('proximity_radius', 1.5)
        self.declare_parameter('pose_smoothing_weight', 0.3)
        
        # Enhanced side-aware safe navigation parameters
        self.declare_parameter('enable_safe_offset', True)
        self.declare_parameter('use_side_aware_offsetting', True)  # Enable dual-side logic
        
        # Left side parameters
        self.declare_parameter('left_offset_distance', 1.1)
        self.declare_parameter('left_min_offset', 0.3)
        self.declare_parameter('left_max_offset', 2.0)
        self.declare_parameter('left_offset_strategy', 'toward_robot')  # 'toward_robot', 'perpendicular_left', 'custom_angle'
        self.declare_parameter('left_custom_angle_deg', -90.0)  # Custom angle in degrees
        
        # Right side parameters
        self.declare_parameter('right_offset_distance', 1.1)
        self.declare_parameter('right_min_offset', 0.3)
        self.declare_parameter('right_max_offset', 2.0)
        self.declare_parameter('right_offset_strategy', 'toward_robot')  # 'toward_robot', 'perpendicular_right', 'custom_angle'
        self.declare_parameter('right_custom_angle_deg', 90.0)  # Custom angle in degrees
        
        # Fallback parameters (when start pose unavailable)
        self.declare_parameter('fallback_offset_distance', 0.9)
        self.declare_parameter('fallback_strategy', 'toward_robot')
        
        # Load basic parameters
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.fallback_frame = self.get_parameter('fallback_frame').get_parameter_value().string_value
        self.transform_timeout = self.get_parameter('transform_timeout').get_parameter_value().double_value
        self.auto_save_starting_pose = self.get_parameter('auto_save_starting_pose').get_parameter_value().bool_value
        self.starting_pose_delay = self.get_parameter('starting_pose_delay').get_parameter_value().double_value
        self.navigation_timeout = self.get_parameter('navigation_timeout').get_parameter_value().double_value
        self.max_navigation_attempts = self.get_parameter('max_navigation_attempts').get_parameter_value().integer_value
        self.proximity_radius = self.get_parameter('proximity_radius').get_parameter_value().double_value
        self.pose_smoothing_weight = self.get_parameter('pose_smoothing_weight').get_parameter_value().double_value
        
        # Load enhanced safe navigation parameters
        self.enable_safe_offset = self.get_parameter('enable_safe_offset').get_parameter_value().bool_value
        self.use_side_aware_offsetting = self.get_parameter('use_side_aware_offsetting').get_parameter_value().bool_value
        
        # Left side parameters
        self.left_offset_distance = self.get_parameter('left_offset_distance').get_parameter_value().double_value
        self.left_min_offset = self.get_parameter('left_min_offset').get_parameter_value().double_value
        self.left_max_offset = self.get_parameter('left_max_offset').get_parameter_value().double_value
        self.left_offset_strategy = self.get_parameter('left_offset_strategy').get_parameter_value().string_value
        self.left_custom_angle_deg = self.get_parameter('left_custom_angle_deg').get_parameter_value().double_value
        
        # Right side parameters
        self.right_offset_distance = self.get_parameter('right_offset_distance').get_parameter_value().double_value
        self.right_min_offset = self.get_parameter('right_min_offset').get_parameter_value().double_value
        self.right_max_offset = self.get_parameter('right_max_offset').get_parameter_value().double_value
        self.right_offset_strategy = self.get_parameter('right_offset_strategy').get_parameter_value().string_value
        self.right_custom_angle_deg = self.get_parameter('right_custom_angle_deg').get_parameter_value().double_value
        
        # Fallback parameters
        self.fallback_offset_distance = self.get_parameter('fallback_offset_distance').get_parameter_value().double_value
        self.fallback_strategy = self.get_parameter('fallback_strategy').get_parameter_value().string_value
    
    def setup_camera_configs(self):
        """Setup camera configurations"""
        self.camera_configs = {
            'front': CameraConfig('/front_cam/aruco/markers', 'panther/camera_front'),
            'rear': CameraConfig('/back_cam/aruco/markers', 'panther/camera_back'),
            'left': CameraConfig('/left_cam/aruco/markers', 'panther/camera_left'),
            'right': CameraConfig('/right_cam/aruco/markers', 'panther/camera_right')
        }
    
    # ==================== Enhanced Safe Pose Calculation Methods ====================
    
    def create_start_pose_reference(self, start_pose: PoseStamped) -> StartPoseReference:
        """Create reference line from starting pose for side determination"""
        yaw = self.quaternion_to_yaw(start_pose.pose.orientation)
        
        reference = StartPoseReference(
            x0=start_pose.pose.position.x,
            y0=start_pose.pose.position.y,
            theta0=yaw,
            cos_theta0=math.cos(yaw),
            sin_theta0=math.sin(yaw)
        )
        
        self.get_logger().info(
            f"🧭 Start pose reference created: [{reference.x0:.2f}, {reference.y0:.2f}], "
            f"yaw: {math.degrees(yaw):.1f}°"
        )
        
        return reference
    
    def determine_marker_side(self, marker_pose: MarkerPose, reference: StartPoseReference) -> tuple:
        """
        CORRECTED: Determine which side of the start pose reference line the marker is on.
        
        Returns:
            tuple: (side_string, cross_product_value)
        """
        xa, ya = marker_pose.x, marker_pose.y
        
        # ✅ CORRECTED CROSS PRODUCT FORMULA
        # For a line defined by point (x0,y0) and direction (cos_theta, sin_theta):
        # Use proper 2D cross product for line-point relationship
        cross = (xa - reference.x0) * (-reference.sin_theta0) + (ya - reference.y0) * reference.cos_theta0
        
        # Positive cross product = left side, negative = right side
        side = 'left' if cross > 0 else 'right'
        
        return side, cross
    
    def calculate_left_side_safe_pose(self, marker_pose: MarkerPose, robot_pose: PoseStamped) -> tuple:
        """Calculate safe pose for markers on the left side of reference line"""
        xa, ya = marker_pose.x, marker_pose.y
        xr, yr = robot_pose.pose.position.x, robot_pose.pose.position.y
        
        # Vector from robot to marker
        vra_x = xa - xr
        vra_y = ya - yr
        vra_magnitude = math.sqrt(vra_x**2 + vra_y**2)
        
        # Determine offset distance with left-side scaling
        if vra_magnitude < self.left_min_offset:
            actual_offset = self.left_min_offset
        elif vra_magnitude > self.left_max_offset:
            actual_offset = self.left_offset_distance
        else:
            scale_factor = vra_magnitude / self.left_max_offset
            actual_offset = max(self.left_min_offset, self.left_offset_distance * scale_factor)
        
        # Calculate offset based on left side strategy
        if self.left_offset_strategy == 'toward_robot':
            # Standard approach: offset toward robot
            if vra_magnitude < 0.01:
                # Edge case: use robot's current orientation
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                offset_x = actual_offset * math.cos(robot_yaw + math.pi)
                offset_y = actual_offset * math.sin(robot_yaw + math.pi)
                safe_yaw = robot_yaw
            else:
                unit_vra_x = vra_x / vra_magnitude
                unit_vra_y = vra_y / vra_magnitude
                offset_x = -actual_offset * unit_vra_x
                offset_y = -actual_offset * unit_vra_y
                safe_yaw = math.atan2(vra_y, vra_x)
                
        elif self.left_offset_strategy == 'perpendicular_left':
            # Offset perpendicular to robot-marker line, to the left
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                offset_x = actual_offset * math.cos(robot_yaw + math.pi/2)
                offset_y = actual_offset * math.sin(robot_yaw + math.pi/2)
                safe_yaw = robot_yaw
            else:
                # Perpendicular vector (rotated 90° counter-clockwise)
                perp_x = -vra_y / vra_magnitude
                perp_y = vra_x / vra_magnitude
                offset_x = actual_offset * perp_x
                offset_y = actual_offset * perp_y
                safe_yaw = math.atan2(vra_y, vra_x)
                
        elif self.left_offset_strategy == 'custom_angle':
            # Offset at custom angle relative to robot-marker vector
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                angle = robot_yaw + math.radians(self.left_custom_angle_deg)
            else:
                base_angle = math.atan2(vra_y, vra_x)
                angle = base_angle + math.radians(self.left_custom_angle_deg)
            
            offset_x = actual_offset * math.cos(angle)
            offset_y = actual_offset * math.sin(angle)
            safe_yaw = math.atan2(vra_y, vra_x) if vra_magnitude > 0.01 else self.quaternion_to_yaw(robot_pose.pose.orientation)
        
        safe_x = xa + offset_x
        safe_y = ya + offset_y
        
        return safe_x, safe_y, safe_yaw, actual_offset
    
    def calculate_right_side_safe_pose(self, marker_pose: MarkerPose, robot_pose: PoseStamped) -> tuple:
        """Calculate safe pose for markers on the right side of reference line"""
        xa, ya = marker_pose.x, marker_pose.y
        xr, yr = robot_pose.pose.position.x, robot_pose.pose.position.y
        
        # Vector from robot to marker
        vra_x = xa - xr
        vra_y = ya - yr
        vra_magnitude = math.sqrt(vra_x**2 + vra_y**2)
        
        # Determine offset distance with right-side scaling
        if vra_magnitude < self.right_min_offset:
            actual_offset = self.right_min_offset
        elif vra_magnitude > self.right_max_offset:
            actual_offset = self.right_offset_distance
        else:
            scale_factor = vra_magnitude / self.right_max_offset
            actual_offset = max(self.right_min_offset, self.right_offset_distance * scale_factor)
        
        # Calculate offset based on right side strategy
        if self.right_offset_strategy == 'toward_robot':
            # Standard approach: offset toward robot
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                offset_x = actual_offset * math.cos(robot_yaw + math.pi)
                offset_y = actual_offset * math.sin(robot_yaw + math.pi)
                safe_yaw = robot_yaw
            else:
                unit_vra_x = vra_x / vra_magnitude
                unit_vra_y = vra_y / vra_magnitude
                offset_x = -actual_offset * unit_vra_x
                offset_y = -actual_offset * unit_vra_y
                safe_yaw = math.atan2(vra_y, vra_x)
                
        elif self.right_offset_strategy == 'perpendicular_right':
            # Offset perpendicular to robot-marker line, to the right
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                offset_x = actual_offset * math.cos(robot_yaw - math.pi/2)
                offset_y = actual_offset * math.sin(robot_yaw - math.pi/2)
                safe_yaw = robot_yaw
            else:
                # Perpendicular vector (rotated 90° clockwise)
                perp_x = vra_y / vra_magnitude
                perp_y = -vra_x / vra_magnitude
                offset_x = actual_offset * perp_x
                offset_y = actual_offset * perp_y
                safe_yaw = math.atan2(vra_y, vra_x)
                
        elif self.right_offset_strategy == 'custom_angle':
            # Offset at custom angle relative to robot-marker vector
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                angle = robot_yaw + math.radians(self.right_custom_angle_deg)
            else:
                base_angle = math.atan2(vra_y, vra_x)
                angle = base_angle + math.radians(self.right_custom_angle_deg)
            
            offset_x = actual_offset * math.cos(angle)
            offset_y = actual_offset * math.sin(angle)
            safe_yaw = math.atan2(vra_y, vra_x) if vra_magnitude > 0.01 else self.quaternion_to_yaw(robot_pose.pose.orientation)
        
        safe_x = xa + offset_x
        safe_y = ya + offset_y
        
        return safe_x, safe_y, safe_yaw, actual_offset
    
    def calculate_enhanced_safe_goal_pose(self, marker_pose: MarkerPose, robot_pose: PoseStamped) -> SafePose:
        """
        Enhanced safe goal pose calculation with side-aware offsetting.
        Uses the robot's starting pose as a reference line to determine which side 
        of the arena the marker is on, then applies appropriate offsetting strategy.
        """
        xa, ya = marker_pose.x, marker_pose.y
        xr, yr = robot_pose.pose.position.x, robot_pose.pose.position.y
        
        # Initialize default values
        side = 'unknown'
        cross_product = 0.0
        offset_method = 'fallback'
        start_reference_used = False
        
        # Determine side if we have starting pose reference and side-aware mode is enabled
        if self.use_side_aware_offsetting and self.start_reference:
            side, cross_product = self.determine_marker_side(marker_pose, self.start_reference)
            start_reference_used = True
            
            # Calculate safe pose based on determined side
            if side == 'left':
                safe_x, safe_y, safe_yaw, actual_offset = self.calculate_left_side_safe_pose(marker_pose, robot_pose)
                offset_method = 'left_side'
            else:  # right side
                safe_x, safe_y, safe_yaw, actual_offset = self.calculate_right_side_safe_pose(marker_pose, robot_pose)
                offset_method = 'right_side'
                
        else:
            # Fallback to single offset method (original behavior)
            vra_x = xa - xr
            vra_y = ya - yr
            vra_magnitude = math.sqrt(vra_x**2 + vra_y**2)
            
            if vra_magnitude < 0.01:
                robot_yaw = self.quaternion_to_yaw(robot_pose.pose.orientation)
                actual_offset = self.fallback_offset_distance
                offset_x = actual_offset * math.cos(robot_yaw + math.pi)
                offset_y = actual_offset * math.sin(robot_yaw + math.pi)
                safe_yaw = robot_yaw
            else:
                actual_offset = self.fallback_offset_distance
                unit_vra_x = vra_x / vra_magnitude
                unit_vra_y = vra_y / vra_magnitude
                offset_x = -actual_offset * unit_vra_x
                offset_y = -actual_offset * unit_vra_y
                safe_yaw = math.atan2(vra_y, vra_x)
            
            safe_x = xa + offset_x
            safe_y = ya + offset_y
            offset_method = 'fallback'
            side = 'fallback'
        
        # Create enhanced SafePose object
        safe_pose = SafePose(
            original_x=xa,
            original_y=ya,
            safe_x=safe_x,
            safe_y=safe_y,
            safe_yaw=safe_yaw,
            offset_distance=actual_offset,
            robot_x=xr,
            robot_y=yr,
            side=side,
            cross_product=cross_product,
            offset_method=offset_method,
            start_reference_used=start_reference_used
        )
        
        # Enhanced logging
        distance_to_marker = math.sqrt((xa - xr)**2 + (ya - yr)**2)
        side_info = f"side: {side}" if start_reference_used else "no side determination"
        method_info = f"method: {offset_method}"
        
        self.get_logger().info(
            f"🛡️ Enhanced safe pose calculated: "
            f"Original=[{xa:.2f}, {ya:.2f}] → Safe=[{safe_x:.2f}, {safe_y:.2f}], "
            f"Offset={actual_offset:.2f}m, {side_info}, {method_info}, "
            f"Robot distance={distance_to_marker:.2f}m"
        )
        
        if start_reference_used:
            self.get_logger().debug(f"🧭 Cross product: {cross_product:.3f} (positive=left, negative=right)")
        
        return safe_pose
    
    def create_safe_pose_stamped(self, safe_pose: SafePose) -> PoseStamped:
        """Create a PoseStamped message from SafePose"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.target_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        pose_stamped.pose.position.x = safe_pose.safe_x
        pose_stamped.pose.position.y = safe_pose.safe_y
        pose_stamped.pose.position.z = 0.0  # Assume ground level
        
        # Convert yaw to quaternion
        half_yaw = safe_pose.safe_yaw / 2.0
        pose_stamped.pose.orientation.w = math.cos(half_yaw)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(half_yaw)
        
        return pose_stamped
    
    def validate_safe_pose(self, safe_pose: SafePose) -> bool:
        """Validate that the safe pose is reasonable"""
        # Basic sanity checks
        if math.isnan(safe_pose.safe_x) or math.isnan(safe_pose.safe_y):
            self.get_logger().error("❌ Safe pose contains NaN values")
            return False
        
        if math.isinf(safe_pose.safe_x) or math.isinf(safe_pose.safe_y):
            self.get_logger().error("❌ Safe pose contains infinite values")
            return False
        
        # Check minimum distance from original marker
        dx = safe_pose.safe_x - safe_pose.original_x
        dy = safe_pose.safe_y - safe_pose.original_y
        actual_offset = math.sqrt(dx**2 + dy**2)
        
        if actual_offset < 0.1:  # Too close to original
            self.get_logger().warn("⚠️ Safe pose too close to original marker position")
            return False
        
        return True
    
    # ==================== Starting Pose Reference Management ====================
    
    def update_start_pose_reference(self):
        """Update the start pose reference when starting pose becomes available"""
        if self.starting_pose and not self.start_reference:
            self.start_reference = self.create_start_pose_reference(self.starting_pose)
            self.get_logger().info("🧭 Start pose reference established for side-aware navigation")
        elif not self.starting_pose and self.use_side_aware_offsetting:
            self.get_logger().warn("⚠️ Side-aware offsetting enabled but no starting pose available - using fallback method")
    
    # ==================== Modified Navigation Methods ====================
    
    def start_navigation_to_marker(self, marker_id: int):
        """Start navigation to marker with enhanced side-aware safe pose offsetting"""
        # Ensure start pose reference is available
        self.update_start_pose_reference()
        
        marker_pose = self.marker_poses[marker_id]
        
        # Determine goal pose based on safe offset setting
        if self.enable_safe_offset:
            # Calculate enhanced safe offset pose
            try:
                safe_pose = self.calculate_enhanced_safe_goal_pose(marker_pose, self.current_robot_pose)
                
                if not self.validate_safe_pose(safe_pose):
                    self.get_logger().error(f"❌ Invalid safe pose calculated for marker {marker_id}")
                    # Fall back to original pose
                    goal_pose = self.create_pose_stamped_from_marker(marker_pose)
                    self.get_logger().info("🔄 Falling back to original marker pose")
                else:
                    goal_pose = self.create_safe_pose_stamped(safe_pose)
                    self.current_safe_pose = safe_pose
                    
                    # Enhanced logging with side information
                    side_info = f" ({safe_pose.side} side via {safe_pose.offset_method})" if safe_pose.start_reference_used else f" (fallback method)"
                    self.get_logger().info(
                        f"🛡️ Using enhanced safe offset pose for marker {marker_id}: "
                        f"[{safe_pose.safe_x:.2f}, {safe_pose.safe_y:.2f}] "
                        f"(offset {safe_pose.offset_distance:.2f}m from original){side_info}"
                    )
                    
            except Exception as e:
                self.get_logger().error(f"❌ Error calculating enhanced safe pose: {e}")
                # Fall back to original pose
                goal_pose = self.create_pose_stamped_from_marker(marker_pose)
                self.current_safe_pose = None
                self.get_logger().info("🔄 Falling back to original marker pose")
        else:
            # Use original marker pose
            goal_pose = self.create_pose_stamped_from_marker(marker_pose)
            self.current_safe_pose = None
            self.get_logger().info(f"📍 Using original marker pose (safe offset disabled)")
        
        # Send navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        # Enhanced logging with side awareness
        pose_type = "enhanced safe offset" if self.current_safe_pose else "original"
        self.get_logger().info(
            f"🚀 Navigating to marker {marker_id} ({pose_type} pose) at "
            f"[{goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}]"
        )
        
        # ✅ FIXED: Actually send the goal to Nav2
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, marker_id))
        
        self.navigation_start_time = time.time()
        self.current_target_id = marker_id
        self.navigation_in_progress = True
    
    # ==================== Communication Setup ====================
    
    def setup_communication(self):
        """Setup publishers, subscribers and services"""
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(Int32, 'navigation_status', reliable_qos)
        
        # Marker subscribers
        self.marker_subscribers = {}
        for camera_name, config in self.camera_configs.items():
            self.marker_subscribers[camera_name] = self.create_subscription(
                MarkerArray, config.topic,
                lambda msg, cam=camera_name: self.marker_callback(msg, cam),
                best_effort_qos
            )
            self.get_logger().info(f"✅ Subscribed to {camera_name}: {config.topic}")
        
        self.get_logger().info(f"🤖 Will get robot pose from TF transform: {self.target_frame} -> {self.robot_frame}")
        
        # Services
        self.get_status_service = self.create_service(
            Trigger, 'get_navigation_status', 
            self.get_navigation_status_callback, 
            callback_group=self.service_callback_group
        )
        
        self.reset_service = self.create_service(
            Trigger, 'reset_navigation',
            self.reset_navigation_callback,
            callback_group=self.service_callback_group
        )
        
        # Enhanced service for reconfiguring side parameters
        self.reconfigure_service = self.create_service(
            Trigger, 'reconfigure_side_parameters',
            self.reconfigure_side_parameters_callback,
            callback_group=self.service_callback_group
        )
    
    def setup_timers(self):
        """Setup timers"""
        self.tf_check_timer = self.create_timer(2.0, self.check_tf_status)
        self.state_timer = self.create_timer(0.5, self.state_machine_tick, 
                                           callback_group=self.navigation_callback_group)
        self.log_timer = self.create_timer(30.0, self.log_status)  # Reduced frequency
        
        # Proximity check timer
        self.proximity_timer = self.create_timer(1.0, self.proximity_check_callback)
        
        # Robot pose update timer
        self.pose_update_timer = self.create_timer(0.1, self.update_robot_pose_from_tf)
        
        # Diagnostic timer - less frequent
        self.diagnostic_timer = self.create_timer(30.0, self.diagnostic_callback)  # Every 30 seconds
        
        if self.auto_save_starting_pose:
            self.starting_pose_timer = self.create_timer(
                self.starting_pose_delay, self.auto_save_starting_pose_callback
            )
    
    # ==================== Robot Pose Management ====================
    
    def update_robot_pose_from_tf(self):
        """Update robot pose using TF transforms"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
            
            new_pose = PoseStamped()
            new_pose.header.frame_id = self.target_frame
            new_pose.header.stamp = self.get_clock().now().to_msg()
            
            new_pose.pose.position.x = transform.transform.translation.x
            new_pose.pose.position.y = transform.transform.translation.y
            new_pose.pose.position.z = transform.transform.translation.z
            new_pose.pose.orientation = transform.transform.rotation
            
            old_pose_available = self.current_robot_pose is not None
            self.current_robot_pose = new_pose
            
            # Log first pose reception only once
            if not old_pose_available:
                x = self.current_robot_pose.pose.position.x
                y = self.current_robot_pose.pose.position.y
                yaw = self.quaternion_to_yaw(self.current_robot_pose.pose.orientation)
                self.get_logger().info(
                    f"🤖 First robot pose received via TF: [{x:.2f}, {y:.2f}], yaw: {yaw:.3f} rad")
            
        except tf2_ros.TransformException:
            # Silent failure - too verbose otherwise
            pass
    
    # ==================== Enhanced Proximity Detection ====================
    
    def check_proximity_to_all_markers(self):
        """Enhanced proximity check that considers both original marker and safe goal positions"""
        if not self.current_robot_pose or not self.marker_poses:
            return
        
        current_time = time.time()
        
        # Skip if checked too recently
        if current_time - self.last_proximity_check_time < self.proximity_check_interval:
            return
        
        if self.state == NavigationState.RETURNING_HOME:
            return
            
        self.last_proximity_check_time = current_time
        
        robot_x = self.current_robot_pose.pose.position.x
        robot_y = self.current_robot_pose.pose.position.y
        
        # Check if robot moved significantly
        robot_moved = True
        if self.last_robot_position:
            dx_robot = robot_x - self.last_robot_position[0]
            dy_robot = robot_y - self.last_robot_position[1]
            move_distance = math.sqrt(dx_robot*dx_robot + dy_robot*dy_robot)
            robot_moved = move_distance > 0.1
        
        self.last_robot_position = (robot_x, robot_y)
        
        if not robot_moved:
            return
        
        # Check proximity - only log occasionally
        should_log = (current_time - self.last_proximity_log_time) > self.proximity_log_interval
        
        with self.data_lock:
            for marker_id, marker_pose in self.marker_poses.items():
                # Calculate distance to original marker position
                dx = robot_x - marker_pose.x
                dy = robot_y - marker_pose.y
                distance_to_marker = math.sqrt(dx*dx + dy*dy)
                
                # If this is the current target and we have a safe pose, also check distance to safe goal
                distance_to_safe_goal = None
                if (marker_id == self.current_target_id and 
                    self.current_safe_pose and 
                    self.enable_safe_offset):
                    dx_safe = robot_x - self.current_safe_pose.safe_x
                    dy_safe = robot_y - self.current_safe_pose.safe_y
                    distance_to_safe_goal = math.sqrt(dx_safe*dx_safe + dy_safe*dy_safe)
                
                # Only log proximity for current target or occasionally for all
                if should_log and (marker_id == self.current_target_id or marker_id == self.required_target_marker):
                    visited_status = "✅" if marker_id in self.visited_markers else "❌"
                    target_status = "🎯" if marker_id == self.current_target_id else ""
                    special_status = "⭐" if marker_id == self.required_target_marker else ""
                    
                    # Enhanced logging with side information
                    side_info = ""
                    if (self.use_side_aware_offsetting and self.start_reference and 
                        distance_to_safe_goal is not None and self.current_safe_pose):
                        side_info = f" ({self.current_safe_pose.side} side)"
                    
                    if distance_to_safe_goal is not None:
                        self.get_logger().info(
                            f"📏 Marker {marker_id}: {distance_to_marker:.2f}m to marker, "
                            f"{distance_to_safe_goal:.2f}m to safe goal{side_info} {visited_status}{target_status}{special_status}"
                        )
                    else:
                        self.get_logger().info(
                            f"📏 Marker {marker_id}: {distance_to_marker:.2f}m {visited_status}{target_status}{special_status}"
                        )
                
                # Check proximity to ORIGINAL marker position for completion
                if (distance_to_marker < self.proximity_radius and 
                    marker_id not in self.visited_markers and
                    marker_id == self.current_target_id):  # Must be current target
                    
                    # This is our current target - check advancement logic
                    should_advance = False
                    
                    if marker_id == self.required_target_marker:
                        # This is marker 0 - always advance (mission complete)
                        should_advance = True
                        self.get_logger().info(
                            f"🎯 SEQUENCE COMPLETE! Final target marker {marker_id} reached at {distance_to_marker:.2f}m"
                        )
                    else:
                        # Check if next marker (n-1) is available
                        next_marker_id = marker_id - 1
                        if next_marker_id >= 0 and next_marker_id in self.marker_poses:
                            # Next marker available - safe to advance
                            should_advance = True
                            self.get_logger().info(
                                f"🎯 SEQUENCE STEP! Marker {marker_id} reached at {distance_to_marker:.2f}m "
                                f"- Next marker {next_marker_id} detected, advancing in sequence"
                            )
                        else:
                            # Next marker not available - just mark as visited but don't advance yet
                            self.visited_markers.add(marker_id)
                            safe_info = f" (via safe offset goal {self.current_safe_pose.side} side at {distance_to_safe_goal:.2f}m)" if (self.current_safe_pose and distance_to_safe_goal is not None) else ""
                            self.get_logger().info(
                                f"🎯 TARGET REACHED! Marker {marker_id} visited at {distance_to_marker:.2f}m{safe_info} "
                                f"- Waiting for marker {next_marker_id} detection before advancing"
                            )
                            # Clear current safe pose since target reached
                            self.current_safe_pose = None
                            # Don't advance - keep navigating to current target
                            continue
                    
                    # Mark as visited
                    self.visited_markers.add(marker_id)
                    # Clear current safe pose
                    self.current_safe_pose = None
                    
                    # Handle advancement ONLY for current target
                    if should_advance:
                        safe_info = f" (via side-aware safe offset navigation)" if distance_to_safe_goal is not None else ""
                        self.get_logger().info(f"🎯 Current target {marker_id} reached via proximity!{safe_info}")
                        self.handle_navigation_success(marker_id)
                
                # INFORMATIONAL ONLY: Log when robot is near non-target markers (but don't visit them)
                elif (distance_to_marker < self.proximity_radius and 
                    marker_id not in self.visited_markers and 
                    marker_id != self.current_target_id and
                    should_log):
                    special_status = "⭐" if marker_id == self.required_target_marker else ""
                    self.get_logger().info(
                        f"ℹ️  Near marker {marker_id} ({distance_to_marker:.2f}m) but not current target - continuing to target {self.current_target_id} {special_status}"
                    )
        
        if should_log:
            self.last_proximity_log_time = current_time
    
    def proximity_check_callback(self):
        """Proximity check with reduced logging"""
        try:
            self.check_proximity_to_all_markers()
        except Exception as e:
            self.get_logger().error(f"Error in proximity check: {e}")
    
    def diagnostic_callback(self):
        """Diagnostic callback - reduced frequency"""
        current_time = time.time()
        if current_time - self.last_diagnostic_log_time < self.diagnostic_log_interval:
            return
        
        self.last_diagnostic_log_time = current_time
        
        self.get_logger().info(f"🔍 DIAGNOSTIC - Robot pose: {'✅' if self.current_robot_pose else '❌'}")
        
        if self.current_robot_pose:
            x = self.current_robot_pose.pose.position.x
            y = self.current_robot_pose.pose.position.y
            self.get_logger().info(f"🔍 Robot at: [{x:.2f}, {y:.2f}]")
        
        self.get_logger().info(f"🔍 Markers: {len(self.marker_poses)}, Visited: {len(self.visited_markers)}")
        self.get_logger().info(f"🔍 Target: {self.current_target_id}, Sequence: {self.navigation_sequence}")
        self.get_logger().info(f"🔍 Mission complete condition: Marker 0 visited = {'✅' if self.required_target_marker in self.visited_markers else '❌'}")
        
        # Enhanced diagnostic info about side-aware offsetting
        if self.enable_safe_offset:
            if self.use_side_aware_offsetting:
                ref_status = "available" if self.start_reference else "not available"
                self.get_logger().info(f"🛡️ Side-aware offset enabled: Left={self.left_offset_distance}m, Right={self.right_offset_distance}m, Reference={ref_status}")
                if self.current_safe_pose:
                    self.get_logger().info(
                        f"🛡️ Current safe pose: [{self.current_safe_pose.safe_x:.2f}, {self.current_safe_pose.safe_y:.2f}] "
                        f"(offset {self.current_safe_pose.offset_distance:.2f}m, {self.current_safe_pose.side} side via {self.current_safe_pose.offset_method})"
                    )
            else:
                self.get_logger().info(f"🛡️ Fallback offset enabled: {self.fallback_offset_distance}m")
        else:
            self.get_logger().info("🛡️ Safe offset disabled")
    
    # ==================== State Machine ====================
    
    def state_machine_tick(self):
        """Main state machine"""
        try:
            status_msg = Int32()
            status_msg.data = self.state.value
            self.status_publisher.publish(status_msg)
            
            # Only log state changes
            if self.last_state_log != self.state:
                self.get_logger().info(f"🔄 State: {self.state.name}")
                self.last_state_log = self.state
            
            if self.state == NavigationState.INITIALIZING:
                self.handle_initialization()
            elif self.state == NavigationState.WAITING_FOR_MARKERS:
                self.handle_waiting_for_markers()
            elif self.state == NavigationState.NAVIGATING_TO_TARGET:
                self.handle_navigation()
            elif self.state == NavigationState.MISSION_COMPLETE:
                self.handle_mission_complete()
            elif self.state == NavigationState.RETURNING_HOME:
                self.handle_returning_home()
            elif self.state == NavigationState.FAILED:
                self.handle_failed_state()
                
        except Exception as e:
            self.get_logger().error(f"State machine error: {e}")
            self.state = NavigationState.FAILED
    
    def handle_initialization(self):
        """Handle initialization with proper sequence logic"""
        if not self.tf_ready:
            return
        
        if not self.marker_poses:
            self.state = NavigationState.WAITING_FOR_MARKERS
            return
        
        # Create navigation sequence from detected markers
        self.update_navigation_sequence()
        
        if self.navigation_sequence:
            self.get_logger().info(f"📋 Navigation sequence: {self.navigation_sequence}")
            self.get_logger().info(f"⭐ Mission goal: Reach marker {self.required_target_marker} (will complete only when marker 0 is visited)")
            if self.required_target_marker not in self.marker_poses:
                self.get_logger().warn(f"⚠️ Required marker {self.required_target_marker} not yet detected - will navigate to available markers")
            self.current_sequence_index = 0
            self.state = NavigationState.NAVIGATING_TO_TARGET
        else:
            self.state = NavigationState.WAITING_FOR_MARKERS
    
    def update_navigation_sequence(self):
        """Create navigation sequence including provision for marker 0"""
        if not self.marker_poses:
            return
        
        # Find highest marker ID
        highest_id = max(self.marker_poses.keys())
        
        # Create sequence: highest down to 0, but only include detected markers
        new_sequence = []
        for marker_id in range(highest_id, -1, -1):
            if marker_id in self.marker_poses:
                new_sequence.append(marker_id)
            elif marker_id == self.required_target_marker:
                # Special case: always expect marker 0 even if not detected yet
                new_sequence.append(marker_id)
        
        # Only update if sequence changed
        if new_sequence != self.navigation_sequence:
            self.navigation_sequence = new_sequence
            self.get_logger().info(f"📋 Updated sequence: {self.navigation_sequence}")
            
            # Log if marker 0 is missing
            if self.required_target_marker not in self.marker_poses:
                self.get_logger().info(f"ℹ️ Marker {self.required_target_marker} not detected yet")
    
    def handle_waiting_for_markers(self):
        """Wait for any markers, not just marker 0"""
        if self.marker_poses:
            self.get_logger().info(f"✅ Markers detected! Starting navigation...")
            self.state = NavigationState.INITIALIZING
    
    def handle_navigation(self):
        """Handle navigation - enhanced with side-aware logging"""
        self.update_navigation_sequence()
        
        # Check if mission complete (only when marker 0 is visited)
        if self.required_target_marker in self.visited_markers:
            self.get_logger().info(f"🏁 Mission complete! Marker {self.required_target_marker} reached!")
            self.state = NavigationState.MISSION_COMPLETE
            return
        
        # Check if we should advance to next available marker when current is visited
        if (self.current_target_id and 
            self.current_target_id in self.visited_markers and 
            not self.navigation_in_progress):
            
            # Current target is visited, check if we can advance
            next_marker_id = self.current_target_id - 1
            if next_marker_id >= 0 and next_marker_id in self.marker_poses:
                # Next marker available - advance
                self.get_logger().info(f"➡️ Current target {self.current_target_id} visited, advancing to {next_marker_id}")
                self.advance_to_next_marker()
                return
            elif next_marker_id < 0:
                # We've gone past marker 0 - check mission completion
                if self.required_target_marker in self.visited_markers:
                    self.state = NavigationState.MISSION_COMPLETE
                    return
                else:
                    # Somehow missed marker 0
                    self.get_logger().warn(f"⚠️ Reached end of sequence but marker {self.required_target_marker} not visited!")
                    self.state = NavigationState.FAILED
                    return
            else:
                # Next marker not detected yet - keep trying current target or wait
                self.get_logger().info(f"⏳ Current target {self.current_target_id} visited, waiting for marker {next_marker_id}")
        
        # Find current target - should be the next unvisited marker in descending order
        current_target = None
        for marker_id in self.navigation_sequence:
            if marker_id not in self.visited_markers:
                current_target = marker_id
                break
        
        # If no unvisited markers in current sequence, handle appropriately
        if current_target is None:
            if self.required_target_marker not in self.marker_poses:
                self.get_logger().info(f"⏳ All available markers visited. Waiting for marker {self.required_target_marker}...")
                return
            else:
                if self.required_target_marker in self.visited_markers:
                    self.state = NavigationState.MISSION_COMPLETE
                    return
                else:
                    self.get_logger().warn(f"⚠️ Navigation sequence exhausted.")
                    self.state = NavigationState.FAILED
                    return
        
        # Skip already visited markers
        if current_target in self.visited_markers:
            return
        
        # Check if we have pose for this target
        if current_target not in self.marker_poses:
            self.get_logger().info(f"⏳ Waiting for marker {current_target} to be detected...")
            return
        
        # Start or check navigation to current target
        if not self.navigation_in_progress:
            self.start_navigation_to_marker(current_target)
        else:
            self.check_navigation_progress(current_target)
    
    def goal_response_callback(self, future, marker_id: int):
        """Handle goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f"❌ Navigation goal rejected for marker {marker_id}")
                self.handle_navigation_failure(marker_id, "goal_rejected")
                return
            
            self.current_nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda future: self.navigation_result_callback(future, marker_id))
            
        except Exception as e:
            self.get_logger().error(f"Goal response error: {e}")
            self.handle_navigation_failure(marker_id, "goal_error")
    
    def navigation_result_callback(self, future, marker_id: int):
        """Handle navigation result with enhanced side-aware logging"""
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.handle_navigation_success(marker_id)
            else:
                self.handle_navigation_failure(marker_id, f"nav_failed_status_{result.status}")
        except Exception as e:
            self.get_logger().error(f"Navigation result error: {e}")
            self.handle_navigation_failure(marker_id, "result_error")
    
    def check_navigation_progress(self, target_id: int):
        """Check navigation timeout"""
        if (self.navigation_start_time and 
            time.time() - self.navigation_start_time > self.navigation_timeout):
            self.get_logger().warn(f"⏱️ Navigation timeout for marker {target_id}")
            self.handle_navigation_failure(target_id, "timeout")
    
    def handle_navigation_success(self, target_id: int):
        """Handle navigation success with enhanced side-aware logging"""
        if target_id not in self.visited_markers:
            self.visited_markers.add(target_id)
            side_info = f" (via {self.current_safe_pose.side} side {self.current_safe_pose.offset_method})" if (self.current_safe_pose and self.current_safe_pose.start_reference_used) else " (via safe offset)" if self.current_safe_pose else ""
            self.get_logger().info(f"✅ Successfully reached marker {target_id}{side_info}")
        else:
            side_info = f" (via {self.current_safe_pose.side} side)" if (self.current_safe_pose and self.current_safe_pose.start_reference_used) else ""
            self.get_logger().info(f"✅ Navigation completed for already-visited marker {target_id}{side_info}")
        
        # Clear safe pose since navigation completed
        self.current_safe_pose = None
        
        # Don't automatically advance here - let handle_navigation() decide
        self.reset_navigation_state()
    
    def handle_navigation_failure(self, target_id: int, reason: str):
        """Enhanced failure handling with side-aware information"""
        attempts = self.navigation_attempts.get(target_id, 0) + 1
        self.navigation_attempts[target_id] = attempts
        
        side_info = f" ({self.current_safe_pose.side} side {self.current_safe_pose.offset_method} goal)" if (self.current_safe_pose and self.current_safe_pose.start_reference_used) else " (safe offset goal)" if self.current_safe_pose else ""
        self.get_logger().warn(f"❌ Navigation failed for marker {target_id}{side_info} (attempt {attempts}/{self.max_navigation_attempts}, reason: {reason})")
        
        # Clear safe pose on failure
        self.current_safe_pose = None
        self.reset_navigation_state()
        
        # Check if we should skip or keep trying
        if attempts >= self.max_navigation_attempts:
            next_marker_id = target_id - 1  # Next marker in sequence (n-1)
            
            # If we have the next marker's pose, skip to it
            if next_marker_id >= 0 and next_marker_id in self.marker_poses:
                self.get_logger().info(f"➡️ Skipping marker {target_id} → moving to marker {next_marker_id} (pose available)")
                self.advance_to_next_available_marker()
            else:
                # Next marker not available, keep trying current marker indefinitely
                self.get_logger().info(f"🔄 Marker {next_marker_id} not detected yet, continuing to try marker {target_id}")
                self.navigation_attempts[target_id] = 0  # Reset attempts to retry indefinitely
    
    def advance_to_next_available_marker(self):
        """Advance to next marker in sequence that has a pose available"""
        self.reset_navigation_state()
        
        # Look for next available marker in sequence
        original_index = self.current_sequence_index
        self.current_sequence_index += 1
        
        # Find next marker with available pose
        while (self.current_sequence_index < len(self.navigation_sequence) and 
               self.navigation_sequence[self.current_sequence_index] not in self.marker_poses):
            self.get_logger().info(f"⏭️ Marker {self.navigation_sequence[self.current_sequence_index]} has no pose, checking next")
            self.current_sequence_index += 1
        
        if self.current_sequence_index < len(self.navigation_sequence):
            next_target = self.navigation_sequence[self.current_sequence_index]
            self.get_logger().info(f"➡️ Moving from marker {self.navigation_sequence[original_index]} to marker {next_target}")
        else:
            self.get_logger().warn("⚠️ No more available markers in sequence")
    
    def reset_navigation_state(self):
        """Reset navigation state"""
        if self.current_nav_goal_handle:
            try:
                self.current_nav_goal_handle.cancel_goal_async()
            except:
                pass
        
        self.current_nav_goal_handle = None
        self.navigation_start_time = None
        self.current_target_id = None
        self.navigation_in_progress = False
        # Clear safe pose when resetting navigation
        self.current_safe_pose = None
    
    def advance_to_next_marker(self):
        """Advance to next marker in sequence"""
        self.reset_navigation_state()
        self.current_sequence_index += 1
    
    def handle_mission_complete(self):
        """Handle mission completion - go home logic"""
        if self.last_state_log != NavigationState.MISSION_COMPLETE:
            self.get_logger().info(f"🎉 Mission complete! Marker {self.required_target_marker} reached!")
            self.get_logger().info(f"📊 Total markers visited: {len(self.visited_markers)} - {sorted(self.visited_markers)}")
            
            # Log side-aware statistics if available
            if self.use_side_aware_offsetting and self.start_reference:
                left_markers = []
                right_markers = []
                for marker_id in self.visited_markers:
                    if marker_id in self.marker_poses:
                        side, _ = self.determine_marker_side(self.marker_poses[marker_id], self.start_reference)
                        if side == 'left':
                            left_markers.append(marker_id)
                        else:
                            right_markers.append(marker_id)
                
                if left_markers or right_markers:
                    self.get_logger().info(f"🧭 Side distribution - Left: {sorted(left_markers)}, Right: {sorted(right_markers)}")
        
        # Go home if we have a starting pose and aren't already navigating
        if self.starting_pose and not self.navigation_in_progress:
            self.state = NavigationState.RETURNING_HOME
            self.start_navigation_home()
        elif not self.starting_pose:
            # No starting pose saved - just stay in mission complete state
            if self.last_state_log != NavigationState.MISSION_COMPLETE:
                self.get_logger().info("🏠 No starting pose available - staying at current location")
    
    def start_navigation_home(self):
        """Navigate home"""
        if not self.nav_action_client.server_is_ready():
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.starting_pose
        
        self.get_logger().info("🏡 Returning to starting position")
        
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.home_goal_response_callback)
        
        self.navigation_start_time = time.time()
        self.navigation_in_progress = True
    
    def home_goal_response_callback(self, future):
        """Handle home goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("🏠 Return home goal rejected")
                return
            
            self.current_nav_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.home_result_callback)
            
        except Exception as e:
            self.get_logger().warn(f"Home goal error: {e}")
    
    def home_result_callback(self, future):
        """Handle home result"""
        try:
            result = future.result()
            if result.status == 4:
                self.get_logger().info("🏠 Successfully returned home!")
            else:
                self.get_logger().warn("🏠 Failed to return home")
        except Exception as e:
            self.get_logger().warn(f"Home result error: {e}")
        finally:
            self.reset_navigation_state()
    
    def handle_returning_home(self):
        """Handle return home timeout"""
        if (self.navigation_start_time and
            time.time() - self.navigation_start_time > self.navigation_timeout):
            self.get_logger().warn("⏱️ Return home timed out")
            self.reset_navigation_state()
    
    def handle_failed_state(self):
        """Handle failed state"""
        if self.last_state_log != NavigationState.FAILED:
            self.get_logger().error("💥 System in failed state. Manual intervention required.")
        self.reset_navigation_state()
    
    # ==================== Marker Detection Callback ====================
    
    def marker_callback(self, msg: MarkerArray, camera_name: str):
        """Process incoming marker detections with enhanced side-aware logging"""
        if not msg.markers:
            return
        
        current_time = time.time()
        camera_config = self.camera_configs.get(camera_name)
        
        if not camera_config:
            return
        
        new_markers_detected = False
        
        with self.data_lock:
            for marker in msg.markers:
                try:
                    marker_id = marker.id
                    
                    # Construct PoseStamped from marker pose
                    marker_pose_stamped = PoseStamped()
                    if hasattr(marker.pose, 'header') and hasattr(marker.pose, 'pose'):
                        marker_pose_stamped.header = marker.pose.header
                        marker_pose_stamped.pose = marker.pose.pose
                    else:
                        marker_pose_stamped.header.frame_id = camera_config.frame_id
                        marker_pose_stamped.header.stamp = rclpy.time.Time().to_msg()
                        marker_pose_stamped.pose = marker.pose
                    
                    # Transform to map frame
                    transformed_pose_stamped = self.tf_buffer.transform(
                        marker_pose_stamped,
                        self.target_frame,
                        timeout=Duration(seconds=self.transform_timeout)
                    )
                    
                    yaw = self.quaternion_to_yaw(transformed_pose_stamped.pose.orientation)
                    
                    # Create new pose
                    new_pose = MarkerPose(
                        x=transformed_pose_stamped.pose.position.x,
                        y=transformed_pose_stamped.pose.position.y,
                        z=transformed_pose_stamped.pose.position.z,
                        yaw=yaw,
                        timestamp=current_time,
                        camera_source=camera_name,
                        detection_count=1
                    )
                    
                    # Apply smoothing if marker exists, otherwise log new detection
                    if marker_id in self.marker_poses:
                        old_pose = self.marker_poses[marker_id]
                        w = self.pose_smoothing_weight
                        new_pose.x = w * new_pose.x + (1 - w) * old_pose.x
                        new_pose.y = w * new_pose.y + (1 - w) * old_pose.y
                        new_pose.z = w * new_pose.z + (1 - w) * old_pose.z
                        new_pose.yaw = w * new_pose.yaw + (1 - w) * old_pose.yaw
                        new_pose.detection_count = old_pose.detection_count + 1
                    else:
                        # Enhanced logging with side information for new markers
                        special = "⭐" if marker_id == self.required_target_marker else ""
                        side_info = ""
                        
                        if self.use_side_aware_offsetting and self.start_reference:
                            side, cross = self.determine_marker_side(new_pose, self.start_reference)
                            side_info = f" [{side} side]"
                        
                        self.get_logger().info(f"📍 New marker {marker_id} detected by {camera_name} at [{new_pose.x:.2f}, {new_pose.y:.2f}]{side_info} {special}")
                        new_markers_detected = True
                    
                    self.marker_poses[marker_id] = new_pose
                        
                except Exception as e:
                    self.get_logger().warn(f"Failed to process marker {getattr(marker, 'id', 'unknown')} from {camera_name}: {e}")
                    continue
    
    # ==================== Utility Methods ====================
    
    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle"""
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w
        
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def create_pose_stamped_from_marker(self, marker_pose: MarkerPose) -> PoseStamped:
        """Create PoseStamped from MarkerPose"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = marker_pose.x
        pose_stamped.pose.position.y = marker_pose.y
        pose_stamped.pose.position.z = marker_pose.z
        
        # Convert yaw to quaternion
        half_yaw = marker_pose.yaw / 2.0
        pose_stamped.pose.orientation.w = math.cos(half_yaw)
        pose_stamped.pose.orientation.z = math.sin(half_yaw)
        
        return pose_stamped
    
    def check_tf_status(self):
        """Check TF system status"""
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame, 
                self.fallback_frame, 
                rclpy.time.Time()
            )
            if not self.tf_ready:
                self.get_logger().info("✅ TF system ready")
            self.tf_ready = True
            
            # Destroy timer once ready
            if hasattr(self, 'tf_check_timer'):
                self.tf_check_timer.destroy()
                
        except Exception:
            self.tf_ready = False
    
    def log_status(self):
        """Enhanced status logging with side-aware information"""
        current_time = time.time()
        if current_time - self.last_status_log_time > 30.0:  # Every 30 seconds
            progress_str = f"{self.current_sequence_index + 1}/{len(self.navigation_sequence)}" if self.navigation_sequence else "0/0"
            
            # Distance to current target
            distance_info = ""
            if self.current_target_id and self.current_robot_pose and self.current_target_id in self.marker_poses:
                marker_pose = self.marker_poses[self.current_target_id]
                dx = self.current_robot_pose.pose.position.x - marker_pose.x
                dy = self.current_robot_pose.pose.position.y - marker_pose.y
                distance = math.sqrt(dx*dx + dy*dy)
                distance_info = f", Distance to target: {distance:.2f}m"
                
                # Add safe goal distance and side info if applicable
                if self.current_safe_pose:
                    dx_safe = self.current_robot_pose.pose.position.x - self.current_safe_pose.safe_x
                    dy_safe = self.current_robot_pose.pose.position.y - self.current_safe_pose.safe_y
                    safe_distance = math.sqrt(dx_safe*dx_safe + dy_safe*dy_safe)
                    side_info = f" {self.current_safe_pose.side} side" if self.current_safe_pose.start_reference_used else ""
                    distance_info += f" (safe goal: {safe_distance:.2f}m{side_info})"
            
            # Mission progress
            mission_status = f", Mission: {'✅' if self.required_target_marker in self.visited_markers else '❌'}"
            
            # Enhanced safe offset status
            if self.use_side_aware_offsetting:
                ref_status = "with reference" if self.start_reference else "no reference"
                safe_status = f", Side-aware offset: ✅ ({ref_status})"
            else:
                safe_status = f", Safe offset: {'✅' if self.enable_safe_offset else '❌'} (fallback mode)"
            
            self.get_logger().info(
                f"📊 Status - State: {self.state.name}, "
                f"Markers: {len(self.marker_poses)}, "
                f"Visited: {len(self.visited_markers)}, "
                f"Target: {self.current_target_id}, "
                f"Progress: {progress_str}{distance_info}{mission_status}{safe_status}"
            )
            self.last_status_log_time = current_time
    
    def save_starting_pose(self):
        """Save current robot pose as starting pose with enhanced side-aware setup"""
        if self.current_robot_pose and not self.starting_pose_saved:
            try:
                self.starting_pose = self.current_robot_pose
                self.starting_pose_saved = True
                
                # Create reference line for side-aware navigation
                if self.use_side_aware_offsetting:
                    self.start_reference = self.create_start_pose_reference(self.starting_pose)
                
                self.get_logger().info(f"✅ Saved starting pose")
                
                if self.use_side_aware_offsetting and self.start_reference:
                    self.get_logger().info("🧭 Created start pose reference for side-aware navigation")
                
                # Stop auto-save timer
                if hasattr(self, 'starting_pose_timer'):
                    self.starting_pose_timer.destroy()
                
            except Exception as e:
                self.get_logger().error(f"Failed to save starting pose: {e}")
    
    def auto_save_starting_pose_callback(self):
        """Auto-save starting pose"""
        if self.current_robot_pose and not self.starting_pose_saved:
            self.save_starting_pose()
    
    # ==================== Enhanced Service Callbacks ====================
    
    def get_navigation_status_callback(self, request, response):
        """Enhanced status service with side-aware information"""
        try:
            # Calculate distances
            marker_distances = {}
            marker_sides = {}
            if self.current_robot_pose:
                robot_x = self.current_robot_pose.pose.position.x
                robot_y = self.current_robot_pose.pose.position.y
                
                for marker_id, marker_pose in self.marker_poses.items():
                    dx = robot_x - marker_pose.x
                    dy = robot_y - marker_pose.y
                    distance = math.sqrt(dx*dx + dy*dy)
                    marker_distances[str(marker_id)] = f"{distance:.2f}m"
                    
                    # Add side information
                    if self.use_side_aware_offsetting and self.start_reference:
                        side, cross = self.determine_marker_side(marker_pose, self.start_reference)
                        marker_sides[str(marker_id)] = f"{side} (cross: {cross:.3f})"
            
            # Enhanced safe pose info
            safe_pose_info = {}
            if self.current_safe_pose:
                safe_pose_info = {
                    'enabled': self.enable_safe_offset,
                    'side_aware_mode': self.use_side_aware_offsetting,
                    'current_safe_x': f"{self.current_safe_pose.safe_x:.2f}",
                    'current_safe_y': f"{self.current_safe_pose.safe_y:.2f}",
                    'offset_distance': f"{self.current_safe_pose.offset_distance:.2f}m",
                    'original_x': f"{self.current_safe_pose.original_x:.2f}",
                    'original_y': f"{self.current_safe_pose.original_y:.2f}",
                    'side': self.current_safe_pose.side,
                    'cross_product': f"{self.current_safe_pose.cross_product:.3f}",
                    'offset_method': self.current_safe_pose.offset_method,
                    'start_reference_used': self.current_safe_pose.start_reference_used
                }
            else:
                safe_pose_info = {
                    'enabled': self.enable_safe_offset,
                    'side_aware_mode': self.use_side_aware_offsetting,
                    'current_safe_pose': None,
                    'left_offset': f"{self.left_offset_distance:.2f}m ({self.left_offset_strategy})",
                    'right_offset': f"{self.right_offset_distance:.2f}m ({self.right_offset_strategy})",
                    'fallback_offset': f"{self.fallback_offset_distance:.2f}m"
                }
            
            # Start reference info
            start_reference_info = {}
            if self.start_reference:
                start_reference_info = {
                    'available': True,
                    'position': f"[{self.start_reference.x0:.2f}, {self.start_reference.y0:.2f}]",
                    'yaw_degrees': f"{math.degrees(self.start_reference.theta0):.1f}°"
                }
            else:
                start_reference_info = {
                    'available': False,
                    'reason': 'No starting pose saved or side-aware mode disabled'
                }
            
            status_info = {
                'state': self.state.name,
                'current_target': self.current_target_id,
                'required_target': self.required_target_marker,
                'mission_complete': self.required_target_marker in self.visited_markers,
                'markers_detected': len(self.marker_poses),
                'markers_visited': len(self.visited_markers),
                'navigation_sequence': self.navigation_sequence,
                'sequence_progress': f"{self.current_sequence_index}/{len(self.navigation_sequence)}",
                'visited_markers': sorted(list(self.visited_markers)),
                'proximity_radius': f"{self.proximity_radius}m",
                'marker_poses': {str(k): f"[{v.x:.2f}, {v.y:.2f}]" for k, v in self.marker_poses.items()},
                'marker_distances': marker_distances,
                'marker_sides': marker_sides,
                'robot_position': f"[{self.current_robot_pose.pose.position.x:.2f}, {self.current_robot_pose.pose.position.y:.2f}]" if self.current_robot_pose else "Unknown",
                'navigation_attempts': self.navigation_attempts,
                'safe_pose_info': safe_pose_info,
                'start_reference_info': start_reference_info
            }
            
            response.success = True
            response.message = json.dumps(status_info, indent=2)
        except Exception as e:
            response.success = False
            response.message = f"Error getting status: {e}"
        return response
    
    def reset_navigation_callback(self, request, response):
        """Reset navigation system"""
        try:
            self.get_logger().info("🔄 Resetting navigation system...")
            
            # Cancel current navigation
            self.reset_navigation_state()
            
            # Reset navigation state but keep marker poses
            self.visited_markers.clear()
            self.navigation_sequence.clear()
            self.current_sequence_index = 0
            self.navigation_attempts.clear()
            
            # Clear safe pose
            self.current_safe_pose = None
            
            # Reset state machine to restart navigation
            if self.marker_poses:
                self.state = NavigationState.INITIALIZING
            else:
                self.state = NavigationState.WAITING_FOR_MARKERS
            self.last_state_log = None
            
            # Reset logging timers
            self.last_status_log_time = 0
            self.last_proximity_log_time = 0
            self.last_diagnostic_log_time = 0
            
            self.get_logger().info(f"✅ Navigation reset complete. Will restart with {len(self.marker_poses)} known markers.")
            
            response.success = True
            response.message = "Navigation system reset successfully"
        except Exception as e:
            response.success = False
            response.message = f"Error resetting navigation: {e}"
        return response
    
    def reconfigure_side_parameters_callback(self, request, response):
        """Service to reconfigure side-aware parameters at runtime"""
        try:
            # Reload parameters from parameter server
            self.declare_and_load_parameters()
            
            # Update start reference if needed
            self.update_start_pose_reference()
            
            # Log new configuration
            if self.use_side_aware_offsetting:
                self.get_logger().info("🔄 Side-aware parameters reconfigured:")
                self.get_logger().info(f"  Left: {self.left_offset_distance}m ({self.left_offset_strategy})")
                self.get_logger().info(f"  Right: {self.right_offset_distance}m ({self.right_offset_strategy})")
                self.get_logger().info(f"  Reference: {'Available' if self.start_reference else 'Not available'}")
            else:
                self.get_logger().info(f"🔄 Side-aware mode disabled, using fallback: {self.fallback_offset_distance}m")
            
            response.success = True
            response.message = "Side-aware parameters reconfigured successfully"
        except Exception as e:
            response.success = False
            response.message = f"Error reconfiguring parameters: {e}"
        return response
    
    # ==================== Shutdown Handling ====================
    
    def destroy_node(self):
        """Enhanced cleanup on shutdown"""
        try:
            # Cancel any ongoing navigation
            if self.current_nav_goal_handle:
                try:
                    self.current_nav_goal_handle.cancel_goal_async()
                    self.get_logger().info("🛑 Cancelled ongoing navigation goal")
                except:
                    pass
            
            self.get_logger().info("👋 Multi-Camera Marker Navigation shutting down gracefully")
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown cleanup: {e}")
        finally:
            super().destroy_node()


# ==================== Main Function ====================

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = MultiCameraMarkerNavigation()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 Starting enhanced multi-camera navigation system with side-aware safe pose offsetting...")
        node.get_logger().info("🧭 Features: Left/Right side determination, Configurable offset strategies, Reference line navigation")
        node.get_logger().info("📍 ArUco Detection: Full marker dictionary support and multi-camera detection")
        
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("⏹️ Shutting down gracefully...")
    except Exception as e:
        node.get_logger().error(f"Critical error: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
            rclpy.shutdown()
        except Exception as e:
            print(f"Error during cleanup: {e}")

if __name__ == '__main__':
    main()
