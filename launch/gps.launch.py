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
from nav2_common.launch import RewrittenYaml

import os

from launch_ros.actions import Node


def generate_launch_description():
    # * Launch configurations

    pkg_share = get_package_share_directory(package_name="gps_waypoint_follower")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    map_subscribe_transient_local = LaunchConfiguration("map_subscribe_transient_local")
    params_file = LaunchConfiguration("params_file")
    namespace = LaunchConfiguration("namespace")

    lifecycle_nodes = ["gps_waypoint_follower"]

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

    declare_bt_xml = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_share, "params", "basic_params.yaml"),
        description="Full path to the ROS2 parameters file to use",
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "default_bt_xml_filename": default_bt_xml_filename,
        "autostart": autostart,
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    # * Nodes
    driver_node = LifecycleNode(
        package="gps_waypoint_follower",
        executable="gps_waypoint_follower",
        name="gps_waypoint_follower",
        namespace="",
        output="screen",
        parameters=[configured_params],
    )

    gps_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_gps_waypoint_follower",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    # ! Robot localization nodes

    params = os.path.join(pkg_share, "params/ekf_gps_2.yaml")

    start_navsat_transform_cmd = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[params, {"use_sim_time": use_sim_time}],
        remappings=[
            # ("imu", "imu/data"), # !!! for bus
            ("gps/fix", "gps/fix"),  # !!! for demo
            ("gps/fix", "imu/nav_sat_fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ],
    )

    # Start robot localization using an Extended Kalman filter...map->odom transform
    start_robot_localization_global_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "odometry/global"),
            ("/set_pose", "/initialpose"),
        ],
    )

    # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
    start_robot_localization_local_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[params, {"use_sim_time": use_sim_time}],
        remappings=[
            ("odometry/filtered", "odometry/local"),
            ("/set_pose", "/initialpose"),
        ],
    )

    # * Create launch description

    ld = LaunchDescription()

    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace)
    ld.add_action(declare_bt_xml)
    ld.add_action(declare_params_file)

    ld.add_action(driver_node)
    ld.add_action(gps_lifecycle)
    ld.add_action(start_navsat_transform_cmd)
    ld.add_action(start_robot_localization_global_cmd)
    ld.add_action(start_robot_localization_local_cmd)

    return ld
