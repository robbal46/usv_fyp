import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


    usv_driver = get_package_share_directory('usv_driver')
    usv_control = get_package_share_directory('usv_control')
    usv_localization = get_package_share_directory('usv_localization')
    usv_navigation = get_package_share_directory('usv_navigation')
    

    return LaunchDescription([

        DeclareLaunchArgument('thrusters', default_value='true'),
        DeclareLaunchArgument('imu', default_value='true'),
        DeclareLaunchArgument('gps', default_value='true'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('yaw_control', default_value='true'),
        DeclareLaunchArgument('surge_control', default_value='true'),

        # Launch sensor drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_driver, 'sensors.launch.py')),
            launch_arguments={
                'imu': LaunchConfiguration('imu'),
                'gps': LaunchConfiguration('gps'),
                'camera': LaunchConfiguration('camera') 
                }.items()
        ),

        # Launch thruster driver (enable SBC - Arduino comms)
        # Only run if on usv platform
        Node(
            package='usv_driver',
            executable='thruster_driver',
            name='thruster_driver',
            condition=IfCondition(LaunchConfiguration('thrusters'))
        ),

        # Launch controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_control, 'control.launch.py')),
            launch_arguments={
                'yaw_control': LaunchConfiguration('yaw_control'),
                'surge_control': LaunchConfiguration('surge_control')
                }.items()
        ),

        # Launch localization
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_localization, 'launch', 'localization.launch.py'))
        ),

        # Launch navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_navigation, 'launch', 'navigation.launch.py'))
        )

    ])