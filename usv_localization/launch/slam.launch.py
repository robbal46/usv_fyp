import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory('usv_localization'), 
    'config', 'tf.rviz')

    orb_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'orb_slam_params.yaml')
    orb_voc = os.path.join(get_package_share_directory('orb_slam2_ros'), 
    'orb_slam2', 'Vocabulary', 'ORBvoc.txt')

    urdf = xacro.process(os.path.join(get_package_share_directory('usv_description'),
    'urdf', 'usv.urdf.xacro'))


    return LaunchDescription([

        DeclareLaunchArgument('offboard', default_value='true'),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='cam_driver',
            namespace='camera',
            parameters=[
                {'video_device': '/dev/video2'},
                {'framerate': 10.0},
                {'io_method': 'mmap'},
                {'frame_id': 'camera_link'},
                {'image_width': 640},
                {'image_height': 480},
                {'camera_name': 'usb_cam'},
                {'camera_info_url': 'package://usv_driver/usb_cam_calib.yaml'}
            ],
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

        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_mono',
            name='orb_slam2',
            parameters=[orb_config, {'voc_file': orb_voc}]
        ),

        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            condition=IfCondition(LaunchConfiguration('offboard'))
        ),

        # Launch rviz with tf view
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('offboard'))
        )

    ])