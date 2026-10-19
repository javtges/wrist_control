"""
apriltag.launch.py — AprilTag detector on the rectified baseplate camera image.

Subscribes:
  /<camera_name>/<image_topic>   (default /tracking_cam/image_rect)
  /<camera_name>/camera_info

Publishes:
  /apriltag/detections   apriltag_msgs/AprilTagDetectionArray (ids, corners, centre)
  TF  camera -> tag36h11:<id>   (or the frame names set in config/tags.yaml)

Launch arguments:
  camera_name   camera namespace (default tracking_cam)
  image_topic   image_rect (default) or image_raw to skip rectification
  tags_config   AprilTagNode parameter file (default config/tags.yaml)
  container     name of an existing container to load the detector into
                (e.g. /tracking_cam/tracking_container from camera.launch.py);
                empty = start a separate container

Running the detector in the camera's container keeps the 1280x720 frames
intra-process; across processes the detector drops frames and the tag flickers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    share = get_package_share_directory('camera_tracking')

    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')
    tags_config = LaunchConfiguration('tags_config')
    container = LaunchConfiguration('container')

    args = [
        DeclareLaunchArgument('camera_name', default_value='tracking_cam'),
        DeclareLaunchArgument('image_topic', default_value='image_rect'),
        DeclareLaunchArgument(
            'tags_config',
            default_value=os.path.join(share, 'config', 'tags.yaml')),
        DeclareLaunchArgument('container', default_value=''),
    ]

    detector = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag',
        namespace='apriltag',
        parameters=[tags_config],
        remappings=[
            ('/apriltag/image_rect', ['/', camera_name, '/', image_topic]),
            ('/apriltag/camera_info', ['/', camera_name, '/camera_info']),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    own_container = ComposableNodeContainer(
        condition=IfCondition(EqualsSubstitution(container, '')),
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[detector],
    )

    load_into_existing = LoadComposableNodes(
        condition=UnlessCondition(EqualsSubstitution(container, '')),
        composable_node_descriptions=[detector],
        target_container=container,
    )

    return LaunchDescription(args + [own_container, load_into_existing])
