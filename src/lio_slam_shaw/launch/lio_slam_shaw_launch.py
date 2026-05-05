import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction


def generate_launch_description():
    def launch_setup(context: LaunchContext):
        params_file = os.path.join(
            get_package_share_directory('lio_slam_shaw'),
            'config',
            'default_params.yaml',
        )

        slam_node = Node(
            package='lio_slam_shaw',
            executable='slam_node',
            name='slam_node',
            output='screen',
            parameters=[params_file]
        )

        return [slam_node]

    # fmt: off
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
