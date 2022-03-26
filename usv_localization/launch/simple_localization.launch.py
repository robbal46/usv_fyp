import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    ### Sensor drivers
    # GPS driver
    gps_driver_node = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps_driver_node',
        namespace='/gps',
        parameters=[
            {'port': '/dev/ttyUSB1'},
            {'frame_id': 'gps_link'}
        ]
    )
    ld.add_action(gps_driver_node)

    # IMU Driver
    imu_driver_node = Node(
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
        ]
    )
    ld.add_action(imu_driver_node)


    ### State estimation nodes - robot_localization stack

    # Navsat transform - takes in GPS fix and outputs odometry
    navsat_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'navsat_params.yaml')
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config],
        remappings=[
            ('/imu', '/imu/data')
        ]
    )
    ld.add_action(navsat_transform_node)

    # Get config for EKF nodes
    ekf_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'simple_ekf_params.yaml')
    # Launch first node for odom -> base_link
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config]
    )
    ld.add_action(ekf_node)




    # Static transform publishers - all sensors need a transform to base_link
    # Could do this with URDF and robot_state_publisher
    gps_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
    )
    ld.add_action(gps_static_tf)
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
    )
    ld.add_action(imu_static_tf)
    cam_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'cam_link']
    )
    ld.add_action(cam_static_tf)

    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config]
    )
    ld.add_action(rviz)

    return ld