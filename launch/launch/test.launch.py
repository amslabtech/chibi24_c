#!/usr/bin/env python3
import launch
from launch_ros.actions import Node
from launch import LaunchService
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess,TimerAction
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # other_launch_file.launch.pyが存在するパッケージ名
    other_launch_file_path = os.path.join(get_package_share_directory("chibi24_c_launch"), 'launch', 'map_pablish.launch.py')
    bagfile_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'bagfiles')
    nap_file_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'map',"map.yaml")


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file_path),
            # launch_arguments={'arg_name': 'arg_value'}.items()  # 必要に応じて引数を渡すことができます
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bagfile_path],
            output='screen'
        )
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()