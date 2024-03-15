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
    other_launch_file_path = os.path.join(get_package_share_directory("chibi24_c_launch"), 'launch', 'chibi_24_c.launch.py')
    bagfile_path = os.path.join(get_package_share_directory('chibi24_c_launch'), 'files', 'bagfiles')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(other_launch_file_path),
            launch_arguments=[("use_sim_time", "True")]
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bagfile_path,'--clock'],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser'])
    ])

if __name__ == '__main__':
    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()