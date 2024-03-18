#!/usr/bin/env python3
import launch
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess,TimerAction
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from distutils.util import strtobool
from launch.launch_description_sources import PythonLaunchDescriptionSource


def setup_node(context, *args, **kwargs):
    use_sim_time_value = bool(strtobool(LaunchConfiguration('use_sim_time').perform(context)))
     # パッケージのディレクトリパスを取得
    localizer_config_path = os.path.join(get_package_share_directory('chibi24_c_localizer'), 'param', 'params.yaml')
    global_path_planner_config_path = os.path.join(get_package_share_directory('chibi24_c_global_path_planner'), 'param', 'params.yaml')
    local_goal_creator_config_path = os.path.join(get_package_share_directory('chibi24_c_local_goal_creator'), 'param', 'params.yaml')
    local_path_planner_config_path = os.path.join(get_package_share_directory('chibi24_c_local_path_planner'), 'param', 'params.yaml')
    obstacle_detector_config_path = os.path.join(get_package_share_directory('chibi24_c_obstacle_detector'), 'config','param', 'params_obstacle_detector.yaml')
    map_file_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'map', "map.yaml") 
    #other_launch_file_path = os.path.join(get_package_share_directory("urg_node2"), 'launch', 'urg_node2.launch.py')


    return [
        Node(
            package='chibi24_c_global_path_planner', 
            executable='chibi24_c_global_path_planner_node',
            output='screen',
            parameters=[global_path_planner_config_path,{'use_sim_time': use_sim_time_value}],
        ),
        Node(
            package='chibi24_c_localizer',
            executable='chibi24_c_localizer_node',
            output='screen',
            parameters=[localizer_config_path,{'use_sim_time': use_sim_time_value}]
        ),
        Node(
            package='chibi24_c_local_goal_creator',
            executable='chibi24_c_local_goal_creator_node',
            output='screen',
            parameters=[local_goal_creator_config_path,{'use_sim_time': use_sim_time_value}]
        ),
        Node(
            package='chibi24_c_local_path_planner',
            executable='chibi24_c_local_path_planner_node',
            output='screen',
            parameters=[local_path_planner_config_path,{'use_sim_time': use_sim_time_value}]
        ),
        Node(
            package='chibi24_c_obstacle_detector',
            executable='chibi24_c_obstacle_detector_node',
            output='screen',
            parameters=[obstacle_detector_config_path,{'use_sim_time': use_sim_time_value}]
        ),
        Node(
            package='chibi24_c_local_map_creator',
            executable='chibi24_c_local_map_creator_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_value}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True, 
            parameters=[{'use_sim_time': use_sim_time_value, 'autostart': True, 'node_names':  ['map_server']}]
        ),
        Node(
            package='roomba_500driver_meiji',
            executable='main500',
            name='main500',
            output='screen'
        ),  
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(other_launch_file_path),
#        )
        
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time if true'
        ),
        OpaqueFunction(function=setup_node),
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()