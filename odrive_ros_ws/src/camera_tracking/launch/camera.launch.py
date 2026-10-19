"""
camera.launch.py — baseplate camera: usb_cam driver + image_proc rectification.

Both run as components in one multi-threaded container (tracking_container)
with intra-process communication, and apriltag.launch.py loads the detector
into the same container. The 1280x800 rgb8 frames are 3 MB each; sent between
processes at 120 fps most of them are dropped (measured ~20 Hz), so the whole
camera -> detector chain stays in one process. External viewers get what the
transport delivers, which is enough for a live view.

After the camera opens, v4l2-ctl forces manual exposure: usb_cam's own
autoexposure/exposure parameters use the old V4L2 control names and have no
effect on the OV9281 (it stayed in aperture-priority mode with a 15.7 ms
exposure, which caps the rate near 60 fps).

Publishes (namespace = camera_name, default tracking_cam):
  /tracking_cam/image_raw     raw frames from usb_cam
  /tracking_cam/camera_info   calibration from config/camera_info.yaml
  /tracking_cam/image_rect    undistorted frames (what AprilTag consumes)

Launch arguments:
  video_device   V4L2 device of the baseplate camera (default /dev/video8, the Arducam OV9281)
  camera_name    namespace of the camera topics (default tracking_cam)
  params_file    usb_cam parameter file (default config/usb_cam.yaml)
  camera_info    calibration yaml (default config/camera_info.yaml)
  exposure       manual exposure in 100 us units (default 50 = 5 ms; one frame
                 at 120 fps is 8.3 ms); 0 = leave the camera's own setting
  start_camera   false = only the rectify container, e.g. when replaying a bag

Example:
  ros2 launch camera_tracking camera.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (EqualsSubstitution, LaunchConfiguration,
                                  NotSubstitution, PythonExpression)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('camera_tracking')

    video_device = LaunchConfiguration('video_device')
    camera_name = LaunchConfiguration('camera_name')
    params_file = LaunchConfiguration('params_file')
    camera_info = LaunchConfiguration('camera_info')
    exposure = LaunchConfiguration('exposure')
    start_camera = LaunchConfiguration('start_camera')

    args = [
        DeclareLaunchArgument('video_device', default_value='/dev/video8'),
        DeclareLaunchArgument('camera_name', default_value='tracking_cam'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'usb_cam.yaml')),
        DeclareLaunchArgument(
            'camera_info',
            default_value=os.path.join(share, 'config', 'camera_info.yaml')),
        DeclareLaunchArgument('exposure', default_value='50'),
        DeclareLaunchArgument('start_camera', default_value='true'),
    ]

    ipc = [{'use_intra_process_comms': True}]

    camera = ComposableNode(
        condition=IfCondition(start_camera),
        package='usb_cam',
        plugin='usb_cam::UsbCamNode',
        name='usb_cam',
        namespace=camera_name,
        parameters=[
            params_file,
            {
                'video_device': video_device,
                'camera_name': camera_name,
                'camera_info_url': PythonExpression(["'file://' + '", camera_info, "'"]),
            },
        ],
        extra_arguments=ipc,
    )

    rectify = ComposableNode(
        package='image_proc',
        plugin='image_proc::RectifyNode',
        name='rectify',
        namespace=camera_name,
        remappings=[('image', 'image_raw')],
        extra_arguments=ipc,
    )

    container = ComposableNodeContainer(
        name='tracking_container',
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[camera, rectify],
    )

    set_exposure = TimerAction(
        period=3.0,
        condition=IfCondition(PythonExpression([
            "'", start_camera, "' == 'true' and ", exposure, " > 0"])),
        actions=[ExecuteProcess(
            cmd=['v4l2-ctl', '-d', video_device,
                 '--set-ctrl=auto_exposure=1',
                 ['--set-ctrl=exposure_time_absolute=', exposure]],
            output='screen',
        )],
    )

    return LaunchDescription(args + [container, set_exposure])
