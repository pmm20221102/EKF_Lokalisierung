from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory('fishbot_navigation2')
    
    # 2. 定义 launch 参数，允许从命令行修改 use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 3. 创建 LaunchConfiguration 对象以便在节点中使用
    use_sim_time_conf = LaunchConfiguration('use_sim_time')

    # 4. 定义 EKF 节点
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        # 将参数传递给节点
        parameters=[
            os.path.join(pkg_share, 'config/ekf.yaml'),
            {'use_sim_time': use_sim_time_conf}
        ],
        # 核心：重映射输出话题，让 SLAM 能找到融合后的数据
        remappings=[('/odometry/filtered', '/odom')]
    )
    
    # 5. 返回 LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        ekf_node
    ])