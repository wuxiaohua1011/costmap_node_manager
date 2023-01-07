# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import os
import launch
from ament_index_python.packages import get_package_share_directory
import launch_ros
from pathlib import Path
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition  # 1
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base_path = Path(get_package_share_directory("costmap_node_manager"))
    default_costmap_config_file_path: Path = base_path / "config" / "config.yaml"

    costmap_config_file_path_arg = launch.actions.DeclareLaunchArgument(
        name="costmap_config_file_path",
        default_value=default_costmap_config_file_path.as_posix(),
    )
    costmap_manager = Node(
        executable="costmap_node_manager",
        package="costmap_node_manager",
        parameters=[
            launch.substitutions.LaunchConfiguration("costmap_config_file_path")
        ],
    )

    should_launch_local_costmap_marker_args = DeclareLaunchArgument(
        "should_launch_local_costmap_marker",
        default_value="False",  # default_value=[], has the same problem
        description="true to start emitting local costmap detected obstacle markers. False by default",
    )
    local_costmap_marker = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d_markers",
        remappings=[
            ("voxel_grid", "/local_costmap/voxel_grid"),
            ("visualization_marker", "/local_costmap/voxel_grid/visualize"),
        ],
        condition=IfCondition(
            LaunchConfiguration("should_launch_local_costmap_marker")
        ),
    )
    ld = launch.LaunchDescription(
        [
            # args
            costmap_config_file_path_arg,
            should_launch_local_costmap_marker_args,
            costmap_manager,
            local_costmap_marker,
        ]
    )
    return ld
