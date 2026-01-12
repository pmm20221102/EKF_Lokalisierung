import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取各个包的共享目录路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    # 获取 AWS 小房子的路径
    aws_house_dir = get_package_share_directory('aws_robomaker_small_house_world')
    
    # 配置文件路径
    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'rviz', 'displaymodel.rviz')
    ekf_config_path = os.path.join(fishbot_navigation2_dir, 'config', 'ekf_vslam.yaml')
    
    # 2. 定义 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # --- A. 启动 AWS 小房子世界 ---
        # 这会启动 Gazebo 并加载带家具的 .world 文件
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(aws_house_dir, 'launch', 'small_house.launch.py')
            )
        ),

        # --- B. 在指定位置生成机器人 (Spawn Robot) ---
        # 避开 (0,0,0) 的墙体，设为客厅空地坐标 [-3.5, 1.0, 0.2]
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'fishbot', 
                '-topic', 'robot_description',
                '-x', '-3.5', 
                '-y', '1.0', 
                '-z', '0.2'
            ],
            output='screen'
        ),

        # --- C. EKF 融合节点 ---
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
        ),

        # --- D. RTAB-Map 视觉 SLAM 节点 ---
        Node(
            package='rtabmap_slam', executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'publish_tf': True,
                'publish_tf_odom': False,
                'wait_imu_to_init': True,
                'database_path': '',
                'RGBD/NeighborLinkRefining': 'true',
                'Mem/IncrementalMemory': 'true',
                'RGBD/SavedLocalizationIgnored': 'true',
                'Rtabmap/DetectionRate': '5.0',
                'Vis/MaxFeatures': '1500',
                'Vis/MinInliers': '12', # 提高门槛保证质量
                'Reg/Force3DoF': 'true', # 强制 2D 修正
                'Mem/UseOdomGravity': 'true', # 启用重力辅助
                'RGBD/OptimizeFromGraphEnd': 'true', # 从轨迹末端优化
            }],
            remappings=[
                ('rgb/image', '/camera_sensor/image_raw'),
                ('depth/image', '/camera_sensor/depth/image_raw'),
                ('rgb/camera_info', '/camera_sensor/camera_info'),
                ('odom', '/odometry/filtered'),
                ('imu', '/imu')
            ]
        ),
        Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_node',
                prefix='xterm -e', # 弹出独立终端
                parameters=[{
                    'stamped': True, # 开启时间戳适配 Jazzy 控制器
                    'frame_id': 'base_link'
                }],
                remappings=[('/cmd_vel', '/cmd_vel')],
                output='screen'
            ),
        # --- E. RViz2 可视化 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])