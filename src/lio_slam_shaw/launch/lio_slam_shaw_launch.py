import os
import socket
import multiprocessing
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    def launch_setup(context: LaunchContext):

        slam_node = Node(
            package='lio_slam_shaw',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[
                os.path.join(get_package_share_directory('lio_slam_shaw'), 'config', 'default_params.yaml')
            ]
        )
        
        return [slam_node]

    # fmt: off
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])