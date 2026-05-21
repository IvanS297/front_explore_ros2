import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к файлу параметров в вашей рабочей области
    # Примечание: Для надежности лучше установить params.yaml в share директорию пакета,
    # но данный путь указывает напрямую на ваш домашний каталог, как в исходном запросе.
    config_file = os.path.expanduser('~/ros2_ws/src/frontier_exploration/config/params.yaml')

    # Узел 1: frontier_exploration
    frontier_exploration_node = Node(
        package='frontier_exploration',
        executable='frontier_exploration',
        name='frontier_exploration',
        parameters=[config_file],
        remappings=[
            ('/odom', '/diff_cont/odom')
        ],
        output='screen'
    )

    # Узел 2: pure_persuit
    pure_pursuit_node = Node(
        package='frontier_exploration',
        executable='pure_persuit',
        name='pure_persuit',
        parameters=[config_file],
        remappings=[
            ('/odom', '/diff_cont/odom'),
            ('/cmd_vel', '/diff_cont/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        frontier_exploration_node,
        pure_pursuit_node
    ])
