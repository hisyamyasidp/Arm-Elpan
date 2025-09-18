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
            executable='camyolo',
            name='camyolo_node',
            output='screen'
        ),
        # Node(
            # package='web_video_server',
            # executable='web_video_server',
            # name='web_video_server',
            # output='screen',
            # parameters=[{
                # "width": 640,
                # "height": 480,
                # "quality": 50
            # }]
        # )
    ])
