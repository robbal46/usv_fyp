import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    lifecycle_nodes = ['map_server',
                       'controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    params_file = os.path.join(get_package_share_directory('usv_navigation'), 
    'params', 'nav2_params.yaml')

    bt_file = os.path.join(get_package_share_directory('usv_navigation'), 
    'params', 'usv_nav2_bt.xml')

    world_file = os.path.join(get_package_share_directory('usv_navigation'), 
    'worlds', 'empty_world.yaml')

    nav2_bringup = os.path.join(get_package_share_directory('nav2_bringup'),
    'launch', 'navigation_launch.py')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'yaml_filename': world_file,
            'default_bt_xml_filename': bt_file,
            #'default_nav_to_pose_bt_xml': bt_file,
            #'default_nav_through_poses_bt_xml': 'invalid_file',            
        },
        convert_types=True
    )

    return LaunchDescription([
        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     name='map_server',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),
        # Node(
        #     package='nav2_recoveries',
        #     executable='recoveries_server',
        #     name='recoveries_server',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),
        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     name='waypoint_follower',
        #     output='screen',
        #     parameters=[params],
        #     remappings=remappings
        # ),        
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_navigation',
        #     output='screen',
        #     parameters=[{'use_sim_time': False},
        #                 {'autostart': True},
        #                 {'node_names': lifecycle_nodes}]
        # )

        # Publish a [likely empty] nav2 map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params],
        ),

        # Manage the lifecycle of map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),

        # Include the rest of Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup),
            launch_arguments={
                'autostart': 'True',
                'params_file': params,
                'map_subscribe_transient_local': 'true',
            }.items(),
        ),


    ])