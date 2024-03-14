#!/usr/bin/env python3
import os
import launch
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # other_launch_file.launch.pyが存在するパッケージ名
    other_launch_file_path = os.path.join(get_package_share_directory("chibi24_c_launch"), 'launch', 'chibi_24_c.launch.py')
    bagfile_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'bagfiles')
    map_file_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'map', "map.yaml")
    
    # nav2_lifecycle_managerの設定
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True, 
        parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': lifecycle_nodes}]
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file_path),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file_path}]
        ),
        start_lifecycle_manager_cmd
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()
