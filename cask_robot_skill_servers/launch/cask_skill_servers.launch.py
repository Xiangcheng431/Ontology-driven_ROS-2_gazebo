"""Start the retained Crane and Jekko skill servers."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='cask_robot_skill_servers', executable='crane_lifting_server',
             name='crane_lifting_server', output='screen', parameters=[{'use_sim_time': True,
                 'physical_cargo_mode': True, 'angular_velocity_control': True,
                 'trolley_velocity_control': True}]),
        Node(package='cask_robot_skill_servers', executable='jekko_skill_server',
             name='jekko_lifting_server', output='screen', parameters=[{'use_sim_time': True}]),
    ])
