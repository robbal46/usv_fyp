# Launches drivers for each sensor

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Config files
    cam_config = os.path.join(get_package_share_directory('usv_localization'), 'config', 'cam_params.yaml')


    return LaunchDescription([

        DeclareLaunchArgument('imu', default_value='True'),
        DeclareLaunchArgument('gps', default_value='False'),
        DeclareLaunchArgument('camera', default_value='False'),


        # IMU Driver
        Node(
            package='bno055',
            executable='bno055',
            name='imu_driver_node',
            parameters=[
                {'uart_port': '/dev/ttyUSB0'},
                {'frame_id': 'imu_link'},
                {'ros_topic_prefix': ''}
            ],
            remappings=[
                ("/imu", "/imu/data"),
                ("/imu_raw", "/imu/data_raw"),
                ('/mag', '/imu/mag'),
                ('/calib_status', '/imu/calib_status')
            ],
            condition=IfCondition(LaunchConfiguration('imu'))
        ),

        # GPS Driver
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='gps_driver_node',
            namespace='/gps',
            parameters=[
                {'port': '/dev/ttyUSB1'},
                {'frame_id': 'gps_link'}
            ],
            condition=IfCondition(LaunchConfiguration('gps'))
        ),

        # Camera Driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='cam_driver',
            namespace='camera',
            parameters=[cam_config],
            condition=IfCondition(LaunchConfiguration('camera'))
        )
            
    ])

