from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='elpan_manual',
            executable='stm32f4',
            name='stm32f4_node',
            output='screen'
        ),
        Node(
            package='elpan_manual',
            executable='stickconnect',
            name='stickconnect_node',
            output='screen'
        ),
    ])
