from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for Nav2 only'
    )
    
    namespace = LaunchConfiguration('namespace')

    # Grid Map Assembler node
    grid_map_assembler_node = Node(
        package='rtabmap_util',
        executable='map_assembler',
        name='grid_map_assembler',
        parameters=[{
            'frame_id': 'map',
            'topic_name': '/map',
            'map_rate': 3.0,          # Lowered from 5Hz to 3Hz to reduce load
            'publish_period': 6.0,
            'grid_size': 0.05,
            'grid_eroded': False,
            'grid_empty_ray_tracing': True,
            'cloud_decimation': 4,
            'cloud_voxel_size': 0.05,
            'cloud_max_depth': 20.0,
            'cloud_min_depth': 0.05,
            'map_filter_radius': 0.0,
            'map_filter_angle': 30.0,
            'map_cleanup': True,
            'incremental_assembly': True,
            'publish_map_odom_tf': False,
            'occupancy_thr': 0.65,
            'prob_hit': 0.75,
            'prob_miss': 0.35,
            'use_sim_time': False,
        }],
        remappings=[
            ('mapData', '/rtabmap/mapData'),
            ('map', '/map'),
        ],
        output='screen'
    )

    # Nav2 launch with passed namespace
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': PathJoinSubstitution([
                FindPackageShare('nav_stack'),
                'config',
                'nav2_params.yaml'
            ]),
            'namespace': namespace
        }.items()
    )
    
    return LaunchDescription([
        declare_namespace_cmd,
        grid_map_assembler_node,
        nav2_launch,
    ])
