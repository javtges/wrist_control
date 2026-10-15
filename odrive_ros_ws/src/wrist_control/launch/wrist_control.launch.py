import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    odrive_share = get_package_share_directory('odrive_can')

    odrive_nodes = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(odrive_share, 'launch', 'wrist_launch.yaml')
        )
    )

    control_node = Node(
        package='wrist_control',
        executable='hsa_odrive_control',
        name='hsa_odrive_control',
        output='screen',
        parameters=[{
            # robot_config defaults to hsa_robots/wrist.yaml
            'control_mode': 3,    # POSITION_CONTROL
            'input_mode': 1,      # PASSTHROUGH
            'vel_ff': 0.0,
            'torque_ff': 0.0,
        }],
    )

    return LaunchDescription([odrive_nodes, control_node])
