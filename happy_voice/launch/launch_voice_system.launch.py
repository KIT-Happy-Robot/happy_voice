from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='happy_tts',
            executable='launch_tts_server',
            name='tts_server'
        ),
        Node(
            package='happy_tts',
            executable='tts',
            name='tts_client'
        ),
        Node(
            package='happy_stt',
            executable='launch_stt_server',  
            name='stt_server'
        ),
        Node(
            package='happy_stt',
            executable='stt',
            name='stt_client'
        ),
        Node(
            package='happy_voice',
            executable='yes_no',
            name='yes_no_node'
        )
    ])
