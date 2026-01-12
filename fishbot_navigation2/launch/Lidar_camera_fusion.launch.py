import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 配置参数
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = launch.substitutions.LaunchConfiguration('map', 
                                default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration('params_file', 
                                default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value=use_sim_time),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path),

        # 1. 启动 depthimage_to_laserscan 节点 (核心修改)
        launch_ros.actions.Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            remappings=[('depth', '/camera_sensor/depth/image_raw'), # 订阅相机深度图话题
                        ('depth_camera_info', '/camera_sensor/depth/camera_info'),
                        ('scan', '/camera_scan')],           # 发布伪激光话题
            parameters=[{
                'scan_height': 10,             # 提取深度图中中间 5 行像素
                'scan_time': 0.033,
                'range_min': 0.1,             # 最小距离
                'range_max': 5.0,             # 最大有效距离
                'output_frame': 'camera_link' # 必须与你机器人的坐标系一致
            }],
            output='screen'
        ),

        # 2. 启动 Nav2 Bringup (包含 Planner, Controller 等)
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),

        # 3. 启动 RViz2
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])