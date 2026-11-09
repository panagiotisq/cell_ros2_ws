"""Launch the CELL localizer node with params.yaml.

    ros2 launch cell_localization localize.launch.py
    ros2 launch cell_localization localize.launch.py params:=/abs/path/to/other.yaml

Then, in another terminal, replay a recording to drive it:
    ros2 bag play <recording.bag>
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('cell_localization'), 'config', 'params.yaml')

    params_arg = DeclareLaunchArgument(
        'params', default_value=default_params,
        description='Path to the params YAML for the localizer node.')

    node = Node(
        package='cell_localization',
        executable='localizer_node',
        name='cell_localization',
        output='screen',
        parameters=[LaunchConfiguration('params')],
    )

    return LaunchDescription([params_arg, node])
