#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import math

class HillSafetyMonitor(Node):
    def __init__(self):
        super().__init__('hill_safety_monitor')

        # Thresholds
        self.z_accel_threshold = 1.0  # m/s^2; tune this based on testing
        self.pitch_threshold_rad = math.radians(10.5)  # 25 degrees

        # State
        self.triggered = False

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Action Client to Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info("🛡️ Hill Safety Monitor node started.")

    def imu_callback(self, msg: Imu):
        if self.triggered:
            return  # Already triggered, no need to act again

        # Acceleration-based detection
        accel_z = msg.linear_acceleration.z

        # Orientation-based detection (use quaternion -> euler)
        pitch = self.get_pitch_from_quaternion(msg.orientation)
# accel_z < self.z_accel_threshold or 
        if abs(pitch) > self.pitch_threshold_rad:
            self.get_logger().warn(f"⚠️ Hill detected! Z-accel={accel_z:.2f}, Pitch={math.degrees(pitch):.1f}°")

            self.stop_rover()
            self.cancel_navigation_goal()
            self.get_logger().error("🛑 Rover is on a hill. For safety, movement is disabled.")
            self.triggered = True

    def stop_rover(self):
        """Publish zero velocity to stop rover."""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def cancel_navigation_goal(self):
        """Cancel active navigation goal (if any)."""
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("⚠️ Nav2 action server not available. Cannot cancel goal.")
            return

        cancel_future = self.nav_client._cancel_goal_async()  # Internal function; may need custom tracking in integrated code

        def cancel_done_callback(fut):
            try:
                result = fut.result()
                if result:
                    self.get_logger().info("✅ Navigation goal canceled successfully.")
                else:
                    self.get_logger().warn("⚠️ Failed to cancel navigation goal.")
            except Exception as e:
                self.get_logger().error(f"❌ Exception while cancelling goal: {e}")

        cancel_future.add_done_callback(cancel_done_callback)

    def get_pitch_from_quaternion(self, q):
        """Convert quaternion to pitch (rotation about Y-axis)."""
        x, y, z, w = q.x, q.y, q.z, q.w
        sinp = +2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        return pitch

def main(args=None):
    rclpy.init(args=args)
    node = HillSafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down Hill Safety Monitor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

