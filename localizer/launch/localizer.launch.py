from launch import LaunchDescription
from launch_ros.actions import Node
import os

pkg_name = 'chibi24_c_localizer'

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        pkg_name,
        'config',
        'param',
        'params.yaml'
    )

    node = Node(
        package='chibi24_c_localizer',
        executable='chibi24_c_localizer_node'
        name='chibi24_c_localizer'
        parameters=[config]
    )

    ld.add_action(node)

    return ld