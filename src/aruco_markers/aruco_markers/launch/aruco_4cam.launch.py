from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("aruco_markers"),  # Your package name
        "config",
        "aruco_params.yaml"
    ])

    camera_names = ["front_cam", "back_cam", "left_cam", "right_cam"]
    # camera_names = ["front_cam" , "back_cam"]

    return LaunchDescription([
        Node(
            package="aruco_markers",
            executable="aruco_markers",
            name="aruco_markers",
            namespace=cam,  # Use camera name for the node
            parameters=[config_file],
            output="screen"
        )
        for cam in camera_names
    ])