from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    follow_node = Node(
            package='one_1',
            executable='follow_node',
            name='follow_node',
            output='screen',
            emulate_tty=True,
        )
    return LaunchDescription([
        follow_node,
    ])
