from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            namespace='rtabmap',
            name='rtabmap',
            parameters=[{
                # Basic ROS2 parameters
                'frame_id': 'panther/base_link',
                'odom_frame_id': '', 
                'map_frame_id': 'map',
                'use_sim_time': False, 
                
                # LiDAR-only subscription parameters
                'subscribe_rgbd': False,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': False, 
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                'approx_sync': True,             
                'visual_odometry': False,
                'odom_sensor_sync': True,        
                
                # CRITICAL: Add queue parameters to fix synchronization issues
                'queue_size': 100,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                
                # Database and memory management
                'delete_db_on_start': True, 
                'database_path': '~/.ros/rtabmap.db',
                
                # Point cloud processing - Optimized for 16-ray LiDAR
                'cloud_voxel_size': 0.05, 
                'cloud_max_depth': 20.0,
                'cloud_min_depth': 0.05,
                'cloud_decimation': 1, 
                
                # RTAB-Map core parameters - Slightly reduced rates
                'Rtabmap/DetectionRate': '3.0',           # Reduced from 5.0 to 3.0
                'Rtabmap/TimeThr': '0',
                'Rtabmap/MemoryThr': '0',
                'Rtabmap/StartNewMapOnLoopClosure': 'false',
                'Rtabmap/CreateIntermediateNodes': 'false', 
                'Rtabmap/MaxRetrieved': '2',               
                'Rtabmap/PublishRAMUsage': 'false',        
                'Rtabmap/PublishMapRate': '3.0',           # Reduced from 5.0 to 3.0
                                                          
                # Memory management - Speed Optimized
                'Mem/RehearsalSimilarity': '0.6',
                'Mem/RecentWmRatio': '0.2',
                'Mem/STMSize': '20',
                'Mem/LaserScanDownsampleStepSize': '2',    
                'Mem/LaserScanVoxelSize': '0.05',
                
                # ICP parameters - Slightly reduced iterations
                'Icp/VoxelSize': '0.075', 
                'Icp/MaxTranslation': '2.0',
                'Icp/MaxRotation': '1.0',
                'Icp/MaxCorrespondenceDistance': '0.2',   
                'Icp/PM': 'true',
                'Icp/PMOutlierRatio': '0.85',
                'Icp/CorrespondenceRatio': '0.15',
                'Icp/Epsilon': '0.001',
                'Icp/MaxIterations': '15',                # Reduced from 20 to 15
                'Icp/PointToPlane': 'true',
                'Icp/PointToPlaneK': '5',
                'Icp/PointToPlaneRadius': '0.0',
                
                # Grid mapping - Fine-tuning for phantom obstacles
                'Grid/FromDepth': 'false', 
                'Grid/3D': 'true', 
                'Grid/CellSize': '0.05',
                'Grid/RangeMax': '20.0',
                'Grid/RangeMin': '0.05',
                'Grid/MaxObstacleHeight': '2.5', 
                'Grid/MaxGroundHeight': '0.18',
                'Grid/MinGroundHeight': '-0.5', 
                'Grid/MaxGroundAngle': '30',
                'Grid/NormalsSegmentation': 'true', 
                'Grid/ClusterRadius': '0.15',
                'Grid/MinClusterSize': '3',              
                'Grid/FlatObstacleDetected': 'true', 
                'Grid/NoiseFilteringRadius': '0.0',
                'Grid/NoiseFilteringMinNeighbors': '1',
                'Grid/PreVoxelFiltering': 'false',
                'Grid/RayTracing': 'true',                 
                'Grid/Scan2dUnknownSpaceFilled': 'true',   
                'Grid/Scan2dMaxFilledRange': '10.0',
                'Grid/FootprintRadius': '0.3',
                
                # ENHANCED gap filling for 16-ray LiDAR (already good)
                'Grid/UnknownSpaceFilled': 'true',
                'Grid/UnknownSpaceFilledRadius': '0.3',
                'Grid/UnknownSpaceFilledMinNeighbors': '1',
                
                # Occupancy grid (for RTAB-Map's own internal grid, not for /map assembler)
                'GridGlobal/MapUpdateRate': '0.0', 
                'Grid/Sensor': '0', 
                'RGBD/ProximityPathMaxNeighbors': '1', 
                
                # Registration parameters
                'Reg/Strategy': '1',
                'Reg/Force3DoF': 'false',
                'Reg/RepeatOnce': 'false',
                
                # Optimizer - Slightly reduced iterations
                'Optimizer/Strategy': '1',
                'Optimizer/Epsilon': '0.0001',
                'Optimizer/Iterations': '8',              # Reduced from 10 to 8
                'Optimizer/Robust': 'false',
                'Optimizer/VarianceIgnored': 'false',
                
                # Loop closure - Speed Optimized
                'LccIcp/Type': '1',
                'LccIcp2/CorrespondenceRatio': '0.01',
                'LccBow/MinInliers': '8',
                'LccBow/InlierDistance': '0.02',
                'LccIcp/MaxTranslation': '1.0',
                'LccIcp/MaxRotation': '0.35',
                
                # Scan parameters - BACK TO ORIGINAL
                'Scan/MaxRange': '20.0',
                'Scan/MinRange': '0.05',
                'Scan/VoxelSize': '0.05',
                'Scan/DownsampleStep': '2',
                'Scan/RangeFiltered': 'false',
                'Scan/FromDepth': 'false',
                
                # Path planning support
                'Path/GlobalRadius': '2.0',
                'Path/MaxRetries': '3',
                'Path/LinearVelocity': '0.5',
                'Path/AngularVelocity': '0.5',
                
                # Localization parameters
                'Localization/Strategy': '1',
                'Localization/MaxRetries': '3',
                
                # CRITICAL: Enable proper map publishing for global costmap
                'PublishTf': 'true',
                'PublishLandmarks': 'false',
                'PublishPdf': 'false',
                'PublishLikelihood': 'false',
                
                # Performance optimization - Enhanced transform handling
                'map_always_update': True,
                'map_empty_ray_tracing': True,
                'cloud_output_voxelized': True, 
                'gen_scan': False,
                'gen_depth': False,
                'wait_for_transform': 0.2,                # Increased from 0.1 to 0.2
                'tf_delay': 0.05,                         # Added transform delay tolerance
                'tf_tolerance': 0.1,                      # Added transform tolerance
            }],
            remappings=[
                ('odom', '/panther/odometry/filtered'),
                ('scan_cloud', '/ground_stabilized_points'),
            ],
            output='screen'
        )
    ])
