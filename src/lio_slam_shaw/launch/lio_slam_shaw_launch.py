import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('lio_slam_shaw'),
        'config',
        'default_params.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the parameter file to use'
    )

    def launch_setup(context: LaunchContext):
        params_file = LaunchConfiguration('params_file').perform(context)

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
        params_file_arg,
        OpaqueFunction(function=launch_setup)
    ])
