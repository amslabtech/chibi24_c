#!/usr/bin/env python3
import launch
from launch_ros.actions import Node
from launch import LaunchService
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess,TimerAction


def generate_launch_description():
     # パッケージのディレクトリパスを取得
    localizer_config_path = os.path.join(get_package_share_directory('chibi24_c_localizer'), 'param', 'params.yaml')
    global_path_planner_config_path = os.path.join(get_package_share_directory('chibi24_c_global_path_planner'), 'param', 'params.yaml')
    local_goal_creator_config_path = os.path.join(get_package_share_directory('chibi24_c_local_goal_creator'), 'param', 'params.yaml')
    local_path_planner_config_path = os.path.join(get_package_share_directory('chibi24_c_local_path_planner'), 'param', 'params.yaml')
    obstacle_detector_config_path = os.path.join(get_package_share_directory('chibi24_c_obstacle_detector'), 'config','param', 'params.yaml')

    return launch.LaunchDescription([
        Node(
            package='chibi24_c_global_path_planner', 
            executable='chibi24_c_global_path_planner_node',
            output='screen',
            parameters=[global_path_planner_config_path]
        ),
        Node(
            package='chibi24_c_localizer',
            executable='chibi24_c_localizer_node',
            output='screen',
            parameters=[localizer_config_path]
        ),
        Node(
            package='chibi24_c_local_goal_creator',
            executable='chibi24_c_local_goal_creator_node',
            output='screen',
            parameters=[local_goal_creator_config_path]
        ),
        Node(
            package='chibi24_c_local_path_planner',
            executable='chibi24_c_local_path_planner_node',
            output='screen',
            parameters=[local_path_planner_config_path]
        ),
        Node(
            package='chibi24_c_obstacle_detector',
            executable='chibi24_c_obstacle_detector_node',
            output='screen',
            parameters=[obstacle_detector_config_path]
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()