"""
Launch file for Cocoa Intent Extractor
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/ttz/LLMAgent-----Cocoa_Speech/ros_ws/src/voice_asr/voice_asr/qwen2.5-7b-instruct-q5_0-00001-of-00002.gguf',
        description='Path to GGUF model file'
    )
    
    gpu_layers_arg = DeclareLaunchArgument(
        'n_gpu_layers',
        default_value='-1',  # Use all GPU layers (RTX 4070 SUPER)
        description='GPU layers (0 for CPU, -1 for all)'
    )
    
    use_cot_arg = DeclareLaunchArgument(
        'use_cot',
        default_value='True',
        description='Enable Chain-of-Thought reasoning (True/False)'
    )
    
    # LLM Backend selection
    llm_backend_arg = DeclareLaunchArgument(
        'llm_backend',
        default_value='llama',  # 'llama' for local, 'openai' for API
        description="LLM backend: 'llama' (local llama.cpp) or 'openai' (OpenAI API)"
    )
    
    openai_model_arg = DeclareLaunchArgument(
        'openai_model',
        default_value='gpt-5.2',  # Fast and cheap, good for testing
        description='OpenAI model to use (gpt-4o-mini, gpt-4o, gpt-4-turbo)'
    )
    
    # Intent Extractor Node
    intent_extractor_node = Node(
        package='cocoa_intent',
        executable='intent_extractor_node',
        name='intent_extractor_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'n_gpu_layers': LaunchConfiguration('n_gpu_layers'),
            'n_ctx': 4096,
            'temperature': 0.0,
            'max_tokens': 1024,
            'input_topic': '/voice/text',
            'output_topic': '/intent',
            'use_cot': LaunchConfiguration('use_cot'),
            # LLM Backend params
            'llm_backend': LaunchConfiguration('llm_backend'),
            'openai_model': LaunchConfiguration('openai_model'),
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        gpu_layers_arg,
        use_cot_arg,
        llm_backend_arg,
        openai_model_arg,
        intent_extractor_node,
    ])
