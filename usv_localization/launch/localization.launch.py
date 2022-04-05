import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro

def generate_launch_description():

    DeclareLaunchArgument('rviz', default_value='false')

    ukf_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'ukf_params.yaml')

    navsat_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'navsat_params.yaml')

    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')

    urdf = xacro.process(os.path.join(get_package_share_directory('usv_description'),
    'urdf', 'usv.urdf.xacro'))

    return LaunchDescription([

        ### State estimation nodes - robot_localization stack        
        
        # Launch first node for odom -> base_link
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_odom_node',
            output='screen',
            parameters=[ukf_config],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/odom')
            ]
        ),

        # Launch another node for map -> odom
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_map_node',
            output='screen',
            parameters=[ukf_config],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/map')
            ]
        ),

        # Navsat transform - takes in GPS fix and outputs odometry    
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_config],
            remappings=[
                ('/imu', '/imu/data'),
                ('/odometry/filtered', 'odometry/filtered/map')
            ]
        ),

        # Robot state publisher to broadcast static tfs from urdf
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': urdf}
            ]
        ),

        # Launch rviz with tf view
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz'))
        )

    ])
