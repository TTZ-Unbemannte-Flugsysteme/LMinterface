import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the standard crazyflie launch file
    crazyflie_package = get_package_share_directory('crazyflie')
    launch_file = os.path.join(crazyflie_package, 'launch', 'launch.py')
    
    # Path to our custom configuration
    config_dir = os.path.join(crazyflie_package, 'config')
    real_config_path = os.path.join(config_dir, 'real_crazyflies.yaml')
    
    # Path to pose bridge script
    scripts_dir = os.path.join(crazyflie_package, 'scripts')
    
    # Arguments
    # We allow switching backend via argument (cflib or cpp)
    backend_arg = DeclareLaunchArgument('backend', default_value='cflib')
    
    # Include the standard launch file with our arguments
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'crazyflies_yaml_file': real_config_path,
            'backend': LaunchConfiguration('backend'),
            'teleop': 'True',
            'mocap': 'True', # Enabled for OptiTrack tracking
            'gui': 'True'
        }.items()
    )
    
    # Pose bridge node - republishes /poses as /cf231/pose
    # This is needed because mocap data comes on /poses but cocoa nodes expect /cf231/pose
    pose_bridge = Node(
        package='crazyflie',
        executable='pose_bridge.py',
        name='pose_bridge',
        parameters=[{
            'drone_ids': ['cf231'],
            'input_topic': '/poses'
        }],
        output='screen'
    )

    return LaunchDescription([
        backend_arg,
        main_launch,
        pose_bridge
    ])
