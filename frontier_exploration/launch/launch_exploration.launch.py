from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_exploration',
            executable='frontier_exploration',
            name='frontier_exploration',
            remappings=[('/odom', '/diff_cont/odom')],
            parameters=[{'debug': True}],
        ),
        Node(
            package='frontier_exploration',
            executable='pure_persuit',
            name='pure_persuit',
            remappings=[
                ('/odom', '/diff_cont/odom'),
                ('/cmd_vel', '/diff_cont/cmd_vel'),
            ],
        ),
    ])
