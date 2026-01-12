import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取配置与参数
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    # 2. SLAM Toolbox 节点 (已优化地图更新频率)
    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
          {'use_sim_time': use_sim_time},
          {
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link', # 请确认你的根坐标系名称
            'scan_topic': '/scan',
            'mode': 'mapping',
            'async_mode': True,
            'map_update_interval': 1.0,           # 适当放慢地图渲染
            'transform_publish_period': 0.05,     # 降低 TF 发布频率减少通信开销 (0.05 = 20Hz)
            'transform_timeout': 0.5,             # 增加等待 TF 的超时时间，防止 RViz 闪烁丢失
            'tf_buffer_duration': 30.0,           # 增大 TF 缓冲区
            'scan_buffer_size': 100,              # 增大扫描缓冲区队列，缓解 "queue is full"
            'scan_buffer_maximum_scan_distance': 10.0, # 限制处理距离，减少匹配计算量
            'minimum_time_interval': 0.2,         # 强制 SLAM 至少间隔 0.2s 处理一帧
          }
        ]
    )

    # 3. RViz2 节点
    start_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 4. 键盘控制节点 (以 xterm 形式弹出，并适配 Jazzy 的 Stamped 消息)
    teleop_node = Node(
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
    )

    # 5. 构建描述符
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(start_rviz_node)
    ld.add_action(teleop_node)

    return ld