from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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

    # Grid Map Assembler node to provide continuous map updates for navigation
    # Add this to your launch file after the RTABMap node

    grid_map_assembler_node = Node(
        package='rtabmap_util',
        executable='map_assembler',
        name='grid_map_assembler',
        parameters=[{
        # Core parameters
            'frame_id': 'map',
            'topic_name': '/map',  # Output topic for nav2
            'map_rate': 5.0,       # 5 Hz publishing rate
            'publish_period': 6.0,
        
            # Grid parameters to match your RTABMap config
            'grid_size': 0.05,     # Match your Grid/CellSize
            'grid_eroded': False,
            'grid_empty_ray_tracing': True,  # Essential for solid appearance
        
        # Performance parameters
            'cloud_decimation': 4,
            'cloud_voxel_size': 0.05,
            'cloud_max_depth': 20.0,
            'cloud_min_depth': 0.05,
        
        # Map bounds
            'map_filter_radius': 0.0,  # No filtering
            'map_filter_angle': 30.0,
            'map_cleanup': True,
        
        # Performance optimization
            'incremental_assembly': True,  # Faster updates
            'publish_map_odom_tf': False,  # RTABMap handles this
        
        # Match your RTABMap occupancy settings
            'occupancy_thr': 0.65,
            'prob_hit': 0.75,
            'prob_miss': 0.35,
        
        # Use sim time
            'use_sim_time': True,
        }],
        remappings=[
        # Input: RTABMap's mapData topic
            ('mapData', '/rtabmap/mapData'),
        # Output: Standard map topic for nav2
            ('map', '/map'),
        ],
        output='screen'
    )

    
    # Launch Nav2 with namespace
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
                'nv2_params.yaml'
            ]),
            'namespace': namespace  # This will use the namespace parameter
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments first
        declare_namespace_cmd,
        
        # Then the launches in proper order
        grid_map_assembler_node,
        nav2_launch,
    ])