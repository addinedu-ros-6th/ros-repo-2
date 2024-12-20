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
    
    aruco_param_dir = LaunchConfiguration(
        'aruco_param',
        default=os.path.join(
            get_package_share_directory('servee_robot'),
            'param',
            'aruco_param.yaml'
        )
    )
    
    obstacle_param_dir = LaunchConfiguration(
        'obstacle_param',
        default=os.path.join(
            get_package_share_directory('servee_robot'),
            'param',
            'obstacle_avoidance.yaml'
        )
    )
    
    robot_info_dir = LaunchConfiguration(
        "robot_info", 
        default=os.path.join(
            get_package_share_directory('servee_robot'),
            'param',
            'robot_info.yaml'
        )
    )
       
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir),
        
        DeclareLaunchArgument(
            'aruco_param',
            default_value=aruco_param_dir),
        
        DeclareLaunchArgument(
            'obstacle_param',
            default_value=obstacle_param_dir),
        
        DeclareLaunchArgument(
            'robot_info',
            default_value=robot_info_dir),
        
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
            parameters=[aruco_param_dir, obstacle_param_dir, robot_info_dir],
            output='screen',
        )
    ])