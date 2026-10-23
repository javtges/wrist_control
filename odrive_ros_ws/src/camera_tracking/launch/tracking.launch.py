"""
tracking.launch.py — full platform-tracking stack:
camera + rectification + AprilTag (one container, intra-process) + platform_pose
+ plate_guard (is the plate mounted as expected?) + tag_monitor.

Run alongside the wrist stack (odrive_can + hsa_odrive_control): platform_pose
needs the joint states to allow /set_platform_zero and tag_monitor records
the motor positions.

Launch arguments:
  video_device     baseplate camera device (default: the Arducam OV9281's /dev/v4l/by-id path)
  platform_config  platform_pose parameters: tag layout, zero file, zeroing guard
                   (default config/platform.yaml)
  tag_id           tag the monitor watches for "lost"; -1 = any tag (default -1)
  output_dir       where tag_monitor writes screenshots (default ~/wrist_captures)
  show_window      open the live OpenCV window (default true)
  image_topic      image_rect (default) or image_raw
  start_camera     false = no usb_cam, e.g. when replaying a bag

Example:
  ros2 launch camera_tracking tracking.launch.py video_device:=/dev/video2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('camera_tracking')

    video_device = LaunchConfiguration('video_device')
    platform_config = LaunchConfiguration('platform_config')
    tag_id = LaunchConfiguration('tag_id')
    output_dir = LaunchConfiguration('output_dir')
    show_window = LaunchConfiguration('show_window')
    image_topic = LaunchConfiguration('image_topic')
    start_camera = LaunchConfiguration('start_camera')

    args = [
        DeclareLaunchArgument('video_device', default_value='/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0'),
        DeclareLaunchArgument(
            'platform_config',
            default_value=os.path.join(share, 'config', 'platform.yaml')),
        DeclareLaunchArgument('tag_id', default_value='-1'),
        DeclareLaunchArgument('output_dir', default_value='~/wrist_captures'),
        DeclareLaunchArgument('show_window', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='image_rect'),
        DeclareLaunchArgument('start_camera', default_value='true'),
    ]

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'camera.launch.py')),
        launch_arguments={
            'video_device': video_device,
            'start_camera': start_camera,
        }.items(),
    )

    # Detector goes into the camera's container (intra-process image_rect).
    apriltag = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'apriltag.launch.py')),
        launch_arguments={
            'image_topic': image_topic,
            'container': '/tracking_cam/tracking_container',
        }.items(),
    )

    platform_pose = Node(
        package='camera_tracking',
        executable='platform_pose',
        name='platform_pose',
        output='screen',
        parameters=[platform_config],
    )

    plate_guard = Node(
        package='camera_tracking',
        executable='plate_guard',
        name='plate_guard',
        output='screen',
        parameters=[platform_config],
    )

    tag_monitor = Node(
        package='camera_tracking',
        executable='tag_monitor',
        name='tag_monitor',
        output='screen',
        parameters=[{
            'tag_id': tag_id,
            'output_dir': output_dir,
            'show_window': show_window,
            'image_topic': ['/tracking_cam/', image_topic],
        }],
    )

    return LaunchDescription(args + [camera, apriltag, platform_pose, plate_guard, tag_monitor])
