import os
from re import I

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    usv_driver = get_package_share_directory('usv_driver')
    usv_control = get_package_share_directory('usv_control')
    usv_localization = get_package_share_directory('usv_localization')
    usv_navigation = get_package_share_directory('usv_navigation')
    

    return LaunchDescription([

        DeclareLaunchArgument('offboard', default_value='false'),
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('control', default_value='true'),
        DeclareLaunchArgument('localization', default_value='true'),
        DeclareLaunchArgument('navigation', default_value='false'),

        # Launch sensor drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_driver, 'sensors.launch.py')),
            launch_arguments={
                'gps': LaunchConfiguration('gps')
            }.items(),
            condition=UnlessCondition(LaunchConfiguration('offboard'))
        ),

        # Launch thruster driver (enable SBC - Arduino comms)
        # Only run if on usv platform
        Node(
            package='usv_driver',
            executable='thruster_driver',
            name='thruster_driver',
            condition=UnlessCondition(LaunchConfiguration('offboard'))
        ),

        # Launch controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_control, 'control.launch.py')),
            launch_arguments={
                'position_control': 'False'
            }.items(),
            condition=IfCondition(LaunchConfiguration('control'))
        ),

        # Launch localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_localization, 'launch', 'localization.launch.py')),
            launch_arguments={
                'rviz': LaunchConfiguration('offboard'),
                'gps': LaunchConfiguration('gps')
            }.items(),
            condition=IfCondition(LaunchConfiguration('localization'))
        ),

        # Start navigation node
        Node(
            package='usv_control',
            executable='nav_to_pose',
            output='screen',
            parameters=[
                {'frame': 'map'}
            ],
            condition=IfCondition(LaunchConfiguration('navigation'))
        )


        

    ])