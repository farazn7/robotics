from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments with default values
    odom_input_arg = DeclareLaunchArgument(
        'odom_input_topic',
        default_value='odometry/filtered',
        description='Input odometry topic to subscribe to'
    )
    
    odom_output_arg = DeclareLaunchArgument(
        'odom_output_topic',
        default_value='odom',
        description='Output odometry topic to publish to'
    )
    
    vel_input_arg = DeclareLaunchArgument(
        'vel_input_topic',
        default_value='cmd_vel',
        description='Input cmd_vel topic to subscribe to'
    )
    
    vel_output_arg = DeclareLaunchArgument(
        'vel_output_topic',
        default_value='panther/cmd_vel',
        description='Output panther/cmd_vel topic to publish to'
    )
    
    imu_input_arg = DeclareLaunchArgument(
        'imu_input_topic',
        default_value='imu/data',
        description='Input IMU topic to subscribe to'
    )
    
    imu_output_arg = DeclareLaunchArgument(
        'imu_output_topic',
        default_value='imu',
        description='Output IMU topic to publish to'
    )
    
    # Create relay nodes using topic_tools
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=[
            LaunchConfiguration('odom_input_topic'),
            LaunchConfiguration('odom_output_topic')
        ]
    )

    vel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='vel_qos_relay',
        arguments=[
            LaunchConfiguration('vel_input_topic'),
            LaunchConfiguration('vel_output_topic')
        ],
        # parameters=[{
        #     "qos_overrides./scan.publisher.reliability": "relaible",
        #     "qos_overrides./scan.publisher.durability": "volatile",
        #     "qos_overrides./scan.publisher.history": "keep_last",
        #     "qos_overrides./scan.publisher.depth": 5,
        # }],
        output='screen'
    )
    
    imu_relay = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        arguments=[
            LaunchConfiguration('imu_input_topic'),
            LaunchConfiguration('imu_output_topic')
        ]
    )
    
    return LaunchDescription([
        odom_input_arg,
        odom_output_arg,
        odom_relay,
        vel_input_arg,
        vel_output_arg,
        vel_relay,
        # imu_input_arg,
        # imu_output_arg,
        # imu_relay
    ])
