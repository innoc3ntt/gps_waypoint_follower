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

import lifecycle_msgs.msg
import os

from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("gps_waypoint_follower")
    parameter_file = LaunchConfiguration("params_file")
    node_name = "minimal_client"

    parameters_file_dir = os.path.join(share_dir, "params")
    parameters_file_path = os.path.join(
        parameters_file_dir, "dual_ekf_navsat_localization.yaml"
    )

    params_declare = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(share_dir, "params", "demo_gps_waypoints.yaml"),
        description="FPath to the ROS2 parameters file to use.",
    )

    driver_node = LifecycleNode(
        package="gps_waypoint_follower",
        executable="client_main",
        name=node_name,
        namespace="",
        output="screen",
        parameters=[parameter_file],
    )

    navsat_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        output="screen",
        parameters=[parameters_file_path],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(params_declare)
    ld.add_action(navsat_node)
    ld.add_action(driver_node)

    return ld
