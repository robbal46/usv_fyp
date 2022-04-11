import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    ukf_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'simple_ukf_params.yaml')

    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')

    urdf = xacro.process(os.path.join(get_package_share_directory('usv_description'),
    'urdf', 'usv.urdf.xacro'))

    return LaunchDescription([

         DeclareLaunchArgument('rviz', default_value='false'),

        ### State estimation nodes - robot_localization stack        
        
        # Launch one node for map -> base_link
        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_node',
            output='screen',
            parameters=[ukf_config]
        ),

        # Add covariance to cmd_vel and republish
        Node(
            package='usv_localization',
            executable='twist_add_covariance',
            parameters=[{'covariance': [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]}
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