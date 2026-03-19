from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nav_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav_stack"),
                "launch",
                "nav_slam.launch.py"
            ])
        )
    )

    topic_relay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav_stack"),
                "launch",
                "topic_relay.launch.py"
            ])
        )
    )

    # relay_scan = Node(
    #     package='nav_stack',
    #     executable='qos_scan',
    #     name='qos_scan',
    #     output='screen'
    # )

    return LaunchDescription([
        # LogInfo(msg="Launching QoS Scan Node..."),
        # relay_scan,
        LogInfo(msg="Launching Topic Relay..."),
        topic_relay,
        LogInfo(msg="Launching SLAM..."),
        nav_slam,
    ])
