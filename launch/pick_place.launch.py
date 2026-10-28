# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    config_package = LaunchConfiguration('moveit_config_package').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    # An empty package name means the default "<robot_name>_moveit_config"
    moveit_config = MoveItConfigsBuilder(
        robot_name, package_name=config_package or None
    ).to_moveit_configs()

    pick_place_node = Node(
        package='qrb_ros_pick_place',
        executable='pick_place_node',
        output='screen',
        parameters=[moveit_config.to_dict(), params_file],
    )

    return [pick_place_node]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('qrb_ros_pick_place'), 'config', 'pick_place.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            description='Robot name used by MoveItConfigsBuilder, e.g. moveit_resources_panda',
        ),
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='',
            description='MoveIt config package of the robot. '
                        'Defaults to <robot_name>_moveit_config when empty',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='YAML file with the pick_place_node parameters',
        ),
        OpaqueFunction(function=launch_setup),
    ])
