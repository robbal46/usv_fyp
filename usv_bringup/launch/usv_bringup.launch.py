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
    

    return LaunchDescription([

        DeclareLaunchArgument('thrusters', default_value='true'),

        # Launch sensor drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(usv_driver, 'launch', 'sensors.launch.py')),
            launch_arguments={
                'imu': 'true',
                'gps': 'true',
                'camera': 'false' 
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
            PythonLaunchDescriptionSource(os.path.join(usv_control, 'launch', 'control.launch.py')),
            launch_arguments={
                'yaw_control': 'true',
                'surge_control': 'false'
                }.items()
        ),




    ])