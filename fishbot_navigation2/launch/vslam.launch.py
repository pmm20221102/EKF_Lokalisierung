import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 获取之前激光雷达的配置路径（可选，部分参数可复用）
    rviz_config_dir = os.path.join(fishbot_navigation2_dir, 'rviz', 'displaymodel.rviz')
    ekf_config_path = os.path.join(fishbot_navigation2_dir, 'config', 'ekf_vslam.yaml')
    nav2_param_path = os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml')

    # 2. 定义 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')

    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path),

        # --- A. IMU 姿态过滤节点 ---
        # 将原始 IMU 数据转化为带四元数姿态的数据
        # Node(
        #     package='imu_filter_madgwick',
        #     executable='imu_filter_madgwick_node',
        #     name='imu_filter',
        #     parameters=[{'use_sim_time': use_sim_time, 'publish_tf': False, 'world_frame': 'enu','use_mag': False,
                        
        # }],
        #     remappings=[('/imu/data_raw', '/imu')], # 根据仿真话题名修改
        #     output='screen'
        # ),

        # --- B. EKF 融合节点 ---
        # 融合编码器 odom、IMU 姿态、以及视觉里程计
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_path, {'use_sim_time': use_sim_time}]
        ),

        # --- C. RTAB-Map 视觉 SLAM 节点 ---
        # 负责发布 map -> odom 并提供全局定位
        Node(
            package='rtabmap_slam', executable='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'frame_id': 'base_link',
                'map_frame_id': 'map',
                'publish_tf': True,            # 由它发布 map -> odom
                'publish_tf_odom': False,      # 设为 False，odom -> base_link 交给 EKF
                'wait_imu_to_init': True,
                'database_path': '',
                'RGBD/NeighborLinkRefining': 'true',
                'Mem/IncrementalMemory': 'true',
                'RGBD/SavedLocalizationIgnored': 'true',
                'Mem/InitMemoryWithId': '0',
                'Rtabmap/DetectionRate': '5.0',
                # 'Vis/MaxFeatures': '1500',
                # 'Vis/MinInliers': '10',
                'Reg/Force3DoF': 'true',

            }],
           remappings=[
                ('rgb/image', '/camera_sensor/image_raw'),
                ('depth/image', '/camera_sensor/depth/image_raw'),
                ('rgb/camera_info', '/camera_sensor/camera_info'),
                ('odom', '/odometry/filtered'),  # 指向 EKF 输出话题
                ('imu', '/imu')            # 指向滤波后的 IMU 话题
            ]
        ),

        # --- D. Nav2 导航堆栈 (不带 AMCL) ---
        # 使用 IncludeLaunchDescription 启动，但不启动 amcl 和 map_server
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/navigation_launch.py']),
        #     launch_arguments={
        #         'use_sim_time': use_sim_time,
        #         'params_file': nav2_param_path}.items(),
        # ),

        # --- E. RViz2 可视化 ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])