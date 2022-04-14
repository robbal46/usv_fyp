import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    orb_config = os.path.join(get_package_share_directory('usv_localization'), 
    'params', 'orb_slam_params.yaml')
    orb_voc = os.path.join(get_package_share_directory('orb_slam2_ros'), 
    'orb_slam2', 'Vocabulary', 'ORBvoc.txt')
    
    return LaunchDescription([

        Node(
            package='orb_slam2_ros',
            executable='orb_slam2_ros_mono',
            name='orb_slam2_pose',
            parameters=[orb_config, {'voc_file': orb_voc}]
        ),

        # Add covariance to ORB SLAM output
        Node(
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

    ])