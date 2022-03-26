import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    cam_config = os.path.join(get_package_share_directory('usv_localization'), 'config', 'cam_params.yaml')
    cam_driver = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='cam_driver',
        namespace='camera',
        parameters=[cam_config]
    )
    ld.add_action(cam_driver)


    orb_config = os.path.join(get_package_share_directory('usv_localization'), 'params', 'orb_slam_params.yaml')
    orb_voc = os.path.join(get_package_share_directory('orb_slam2_ros'), 'orb_slam2', 'Vocabulary', 'ORBvoc.txt')
    orb_slam = Node(
        package='orb_slam2_ros',
        executable='orb_slam2_ros_mono',
        name='orb_slam2_mono',
        parameters=[orb_config, {'voc_file': orb_voc}]
    )
    ld.add_action(orb_slam)

    # Add covariance to ORB SLAM output
    pose_cov = Node(
        package='usv_localization',
        executable='pose_add_covariance',
        name='pose_add_covariance',
        parameters=[{'covariance': [0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.01, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.01]}
        ]
    )
    ld.add_action(pose_cov)

    return ld