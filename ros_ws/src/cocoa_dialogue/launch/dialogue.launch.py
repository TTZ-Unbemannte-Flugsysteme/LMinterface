"""
Simple launch file for Cocoa Dialogue
Just launches the dialogue manager for speech-to-text
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    ptt_key_arg = DeclareLaunchArgument(
        'ptt_key',
        default_value='space',
        description='Push-to-talk key'
    )
    
    whisper_device_arg = DeclareLaunchArgument(
        'whisper_device',
        default_value='cuda',
        description='Whisper device (cuda or cpu)'
    )
    
    # Dialogue Manager Node
    dialogue_manager_node = Node(
        package='cocoa_dialogue',
        executable='dialogue_manager_node',
        name='dialogue_manager_node',
        output='screen',
        parameters=[{
            'ptt_key': LaunchConfiguration('ptt_key'),
            'whisper_device': LaunchConfiguration('whisper_device'),
            'whisper_model': 'base.en',
            'output_topic': '/voice/text',
        }]
    )
    
    return LaunchDescription([
        ptt_key_arg,
        whisper_device_arg,
        dialogue_manager_node,
    ])
