import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mv'),
        'config', 'config.yaml'
    )

    load_nodes=GroupAction(
        actions=[
            Node(
                package='mv',
                executable='mv_node',
                output='screen',
                parameters=[config],
                emulate_tty=True,
                prefix='gnome-terminal -- gdb -ex "set logging file ./debug/gdb/mv_gdb.txt" -ex "set logging enabled" --args'
            ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld