# Launches controllers for usv (yaw & surge)

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
    
        DeclareLaunchArgument('yaw_control', default_value='True'),
        DeclareLaunchArgument('surge_control', default_value='True'),
        DeclareLaunchArgument('position_control', default_value='False'),

        # Yaw Controller
        Node(
            package='usv_control',
            executable='yaw_controller',
            name='yaw_controller',
            output='screen',
            parameters=[
                {'pid': [10.0, 0.0, 0.0]}
            ],
            condition=IfCondition(LaunchConfiguration('yaw_control'))
        ),        

        # Surge Controller
        Node(
            package='usv_control',
            executable='surge_controller',
            name='surge_controller',
            output='screen',
            parameters=[
                {'order': 3}
            ],
            condition=IfCondition(LaunchConfiguration('surge_control'))
        ),

        # Yaw velocity controller
        Node(
            package='usv_control',
            executable='velocity_pid_controller',
            parameters=[
                {'pid': [1.0, 0.0, 0.0]},
                {'relative': False}
            ],
            remappings=[
                ('/cmd_vel/yaw', '/cmd_vel')
            ],
            condition=IfCondition(LaunchConfiguration('position_control'))
        )   

    ])
    