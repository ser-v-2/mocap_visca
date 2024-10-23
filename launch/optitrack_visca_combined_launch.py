from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import sys
import os

sys.path.append(os.path.abspath('/home/ser_v/tfg/docker/workspace/src/optitrack_visca/optitrack_visca'))
def generate_launch_description():
    camera_params = LaunchConfiguration('camera_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params',
            default_value='camera_params.yaml',
            description='Configuration file for the camera'
        ),
        Node(
            package='optitrack_visca',
            executable='combined_node',
            name='combined_node',
            output='screen',
            parameters=[PathJoinSubstitution([
                FindPackageShare('optitrack_visca'),
                'config',
                'camera_params.yaml'
            ])],
            emulate_tty=True,
        )
    ])
