import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from functools import partial
import os
import time


class AnomalyDetectorNode(Node):
    """
    A ROS2 node that detects anomalies from multiple camera feeds.
    When an anomaly is detected, it creates a circular suppression zone around the
    rover's current position, pausing detection from that specific camera until
    the rover has moved outside the zone.
    """
    def __init__(self):
        super().__init__('anomaly_detector_node')

        # --- Parameters ---
        self.declare_parameter('suppression_radius', 1.0)  # The 'x' meters for the zone
        self.suppression_radius = self.get_parameter('suppression_radius').get_parameter_value().double_value
        self.get_logger().info(f"Suppression radius set to: {self.suppression_radius} meters")

        # --- CV and ROS Tools ---
        self.bridge = CvBridge()

        # --- State Management ---
        self.camera_topics = {
            'front_cam': '/panther/camera_front/image_raw',
            'rear_cam': '/panther/camera_back/image_raw',
            'left_cam': '/panther/camera_left/image_raw',
            'right_cam': '/panther/camera_right/image_raw'
        }
        self.anomaly_positions = {cam: [] for cam in self.camera_topics}
        self.detection_suppressed = {cam: False for cam in self.camera_topics}
        self.last_detection_pos = {cam: None for cam in self.camera_topics}
        self.current_position = None
        self.latest_frames = {cam: None for cam in self.camera_topics}  # store frames for display in main thread

        # --- Subscriptions ---
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odometry_callback,
            10
        )

        for camera_name, topic in self.camera_topics.items():
            self.create_subscription(
                Image,
                topic,
                partial(self.image_callback, camera_name=camera_name),
                10
            )
            self.get_logger().info(f"Subscribed to {camera_name} on topic {topic}")

        self.get_logger().info("Color detector node started")

        # Trackbar window
        cv2.namedWindow("Tracking")
        cv2.createTrackbar("Lower Hue", "Tracking", 0, 255, lambda x: None)
        cv2.createTrackbar("Lower Sat", "Tracking", 0, 255, lambda x: None)
        cv2.createTrackbar("Lower Val", "Tracking", 70, 255, lambda x: None)
        cv2.createTrackbar("Upper Hue", "Tracking", 255, 255, lambda x: None)
        cv2.createTrackbar("Upper Sat", "Tracking", 255, 255, lambda x: None)
        cv2.createTrackbar("Upper Val", "Tracking", 255, 255, lambda x: None)

        # --- Image Saving ---
        self.output_dir = "anomaly_images"
        os.makedirs(self.output_dir, exist_ok=True)
        self.get_logger().info(f"Saving detected anomaly images to: {os.path.abspath(self.output_dir)}")

    def odometry_callback(self, msg):
        self.current_position = msg.pose.pose.position

    def estimate_anomaly_position(self, camera_name):
        if not self.current_position:
            return None
        anomaly_pos = Point()
        anomaly_pos.x = self.current_position.x + 1.0
        anomaly_pos.y = self.current_position.y
        anomaly_pos.z = self.current_position.z
        return anomaly_pos

    def image_callback(self, msg, camera_name):
        if self.detection_suppressed[camera_name]:
            if self.current_position and self.last_detection_pos[camera_name]:
                last_pos = self.last_detection_pos[camera_name]
                dist = np.sqrt((self.current_position.x - last_pos.x) ** 2 +
                               (self.current_position.y - last_pos.y) ** 2)
                if dist > self.suppression_radius:
                    self.get_logger().info(f"Rover has left suppression zone for {camera_name}. Resuming detection.")
                    self.detection_suppressed[camera_name] = False
                    self.last_detection_pos[camera_name] = None
                else:
                    return
            else:
                return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("Lower Hue", "Tracking")
        l_s = cv2.getTrackbarPos("Lower Sat", "Tracking")
        l_v = cv2.getTrackbarPos("Lower Val", "Tracking")
        u_h = cv2.getTrackbarPos("Upper Hue", "Tracking")
        u_s = cv2.getTrackbarPos("Upper Sat", "Tracking")
        u_v = cv2.getTrackbarPos("Upper Val", "Tracking")

        lower_mars_hsv = np.array([l_h, l_s, l_v])
        upper_mars_hsv = np.array([u_h, u_s, u_v])

        mars_mask = cv2.inRange(hsv, lower_mars_hsv, upper_mars_hsv)
        anomaly_mask = cv2.bitwise_not(mars_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        processed_mask = cv2.morphologyEx(anomaly_mask, cv2.MORPH_OPEN, kernel)
        processed_mask = cv2.morphologyEx(processed_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(processed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        anomaly_found_in_frame = False
        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                anomaly_found_in_frame = True
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, 'ANOMALY', (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if anomaly_found_in_frame:
            self.get_logger().info(f"ANOMALY DETECTED by {camera_name}!")
            if self.current_position:
                anomaly_pos = self.estimate_anomaly_position(camera_name)
                if anomaly_pos:
                    self.anomaly_positions[camera_name].append(anomaly_pos)
                    self.get_logger().info(
                        f"Estimated anomaly position for {camera_name}: "
                        f"x={anomaly_pos.x:.2f}, y={anomaly_pos.y:.2f}, z={anomaly_pos.z:.2f}"
                    )

                self.get_logger().info(f"Pausing detection for {camera_name}. Suppression zone of {self.suppression_radius}m activated.")
                self.detection_suppressed[camera_name] = True
                self.last_detection_pos[camera_name] = self.current_position

                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = os.path.join(self.output_dir, f"{camera_name}_anomaly_{timestamp}.jpg")
                cv2.imwrite(filename, frame)
                self.get_logger().info(f"Saved anomaly image to {filename}")
            else:
                self.get_logger().warn("Anomaly detected, but no odometry data available to set suppression zone.")

        # Instead of showing here, store for display in main thread
        self.latest_frames[camera_name] = frame


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            # Show images for all cameras from main thread
            for cam, frame in node.latest_frames.items():
                if frame is not None:
                    cv2.imshow(f"Output - {cam}", frame)
            cv2.waitKey(1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

