import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # EKF params
    ekf_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'simple_ekf_params.yaml')

    # Navsat transform params
    navsat_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'navsat_params.yaml')

    # Rviz config
    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')

    return LaunchDescription([


        # Launch first node for odom -> base_link
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config]
        ),


        # Navsat transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('/imu', '/imu/data')
            ]
        ),             


        # Static transform publishers - all sensors need a transform to base_link
        # Could do this with URDF and robot_state_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )


    ])