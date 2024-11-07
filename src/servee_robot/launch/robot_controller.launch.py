import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('servee_robot'),
            'param',
            'battery_config.yaml'
        )
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir),
        
        Node(
            package='servee_robot',
            executable='battery_state_pub',
            name='battery_state_pub',
            parameters=[param_dir],
            output='screen'
        ),
          
        Node(
            package='servee_robot',
            executable='robot_trees',
            output='screen',
        )
    ])