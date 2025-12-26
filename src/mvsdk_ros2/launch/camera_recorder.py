#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='recorded_images',
        description='Output directory for recorded images'
    )

    image_format_arg = DeclareLaunchArgument(
        'image_format',
        default_value='jpg',
        description='Image format for recorded images (jpg, png, etc.)'
    )

    auto_start_recording_arg = DeclareLaunchArgument(
        'auto_start_recording',
        default_value='false',
        description='Whether to automatically start recording when node starts'
    )

    # 相机节点
    camera_node = Node(
        package='mvsdk_ros2',
        executable='mvsdk_ros2_node',
        name='mv_camera',
        output='screen',
        parameters=[
            # 可以在这里添加相机参数, 但是现在还没写
        ]
    )

    # 录制节点
    recorder_node = Node(
        package='mvsdk_ros2',
        executable='mv_recorder_node',
        name='mv_recorder',
        output='screen',
        parameters=[
            {'output_dir': LaunchConfiguration('output_dir')},
            {'image_format': LaunchConfiguration('image_format')},
            {'auto_start_recording': LaunchConfiguration('auto_start_recording')}
        ]
    )

    return LaunchDescription([
        output_dir_arg,
        image_format_arg,
        auto_start_recording_arg,
        camera_node,
        recorder_node
    ])
