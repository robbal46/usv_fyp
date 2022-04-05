import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    urdf = xacro.process(os.path.join(get_package_share_directory('usv_description'),
    'urdf', 'usv.urdf.xacro'))

    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')

    return LaunchDescription([

        # Robot state publisher to broadcast static tfs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': urdf}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )

    ])

