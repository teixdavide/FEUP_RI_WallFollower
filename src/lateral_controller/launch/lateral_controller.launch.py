import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lateral_controller'),
        '..config',
        'lateral_controller.yaml'
    )

    return LaunchDescription([
        Node(
            package='lateral_controller',
            executable='lateral_controller',
            name='lateral_controller',
            output='screen',
            parameters=[config_file]
        )
    ])