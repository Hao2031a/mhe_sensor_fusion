"""Standalone Gazebo MHE A/B launch. Stop any other odom TF publisher first."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('mhe_sensor_fusion'), 'config',
        'mhe_gazebo_low_latency.yaml'])
    return LaunchDescription([
        Node(
            package='mhe_sensor_fusion',
            executable='mhe_sensor_fusion_node',
            name='mhe_sensor_fusion',
            output='screen',
            parameters=[config],
        )
    ])
