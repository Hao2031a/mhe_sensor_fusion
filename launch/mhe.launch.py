from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mhe_sensor_fusion')
    params = os.path.join(pkg_share, 'config', 'mhe.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock from simulation'
        ),
        Node(
            package='mhe_sensor_fusion',
            executable='mhe_sensor_fusion_node',
            name='mhe_sensor_fusion',
            output='screen',
            parameters=[
                params,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        )
    ])
