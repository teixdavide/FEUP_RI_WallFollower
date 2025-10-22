import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = get_package_share_directory('ri_assign1')
    robot_description_path = os.path.join(package_dir, 'resource', 'my_robot.urdf')

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'my_world.wbt')
    )

    my_robot_driver = WebotsController(
        robot_name='my_robot',
        parameters=[{'robot_description': robot_description_path}],
        namespace='my_robot',
    )
    follower_driver = WebotsController(
        robot_name='follower',
        parameters=[{'robot_description': robot_description_path}],
        namespace='follower',
    )

    leader = Node(
        package='ri_assign1',
        executable='wall_follower',
        namespace='leader',
        parameters=[
            {"desired_distance": 0.7},
            {"kp": 1.0},
            {"ki": 0.0},
            {"kd": 0.1},
            {"max_angular_speed": 0.5},
            {"forward_speed": 0.1},
            {"scan_topic": "/my_robot/scan"},
            {"cmd_vel_topic": "/my_robot/cmd_vel"},
            {"is_leader": True},
            {"leader_detect_threshold": 0.5},
            {"leader_detect_half_deg": 20.0},
        ],
    )
    follower = Node(
        package='ri_assign1',
        executable='wall_follower',
        namespace='follower',
        parameters=[
            {"desired_distance": 0.7},
            {"kp": 1.0},
            {"ki": 0.0},
            {"kd": 0.1},
            {"max_angular_speed": 0.5},
            {"forward_speed": 0.1},
            {"scan_topic": "/follower/scan"},
            {"cmd_vel_topic": "/follower/cmd_vel"},
            {"use_sinusoidal_speed": True},
            {"sin_speed_amplitude": 0.05},
            {"sin_speed_period": 4.0},
        ],
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        follower_driver,
        leader,
        follower,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])