# Copyright (c) 2020 Fetullah Atas
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
from launch.event_handlers.on_shutdown import OnShutdown
from launch_ros.substitutions import FindPackageShare


import lifecycle_msgs.msg
import os

from launch_ros.actions import Node


def generate_launch_description():
    # * Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")

    # * Declares
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    # * Nodes
    driver_node = LifecycleNode(
        package="gps_waypoint_follower",
        executable="gps_waypoint_follower",
        name="gps_waypoint_follower",
        namespace="",
        output="screen",
    )
    gps_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_gps_waypoint_follower",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["gps_waypoint_follower"]},
        ],
    )

    # * Create launch description

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(driver_node)
    ld.add_action(gps_lifecycle)
    return ld
