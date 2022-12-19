# Author: Addison Sears-Collins
# Date: September 2, 2021
# Description: Launch a basic mobile robot using the ROS 2 Navigation Stack
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    nav2_dir = get_package_share_directory("nav2_bringup")
    pkg_share = get_package_share_directory("gps_waypoint_follower")

    autostart = LaunchConfiguration("autostart")
    map_yaml_file = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    slam = LaunchConfiguration("slam")
    use_namespace = LaunchConfiguration("use_namespace")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value="info", description="log level for nodes"
            ),
            DeclareLaunchArgument(
                name="namespace", default_value="", description="Top-level namespace"
            ),
            DeclareLaunchArgument(
                name="use_namespace",
                default_value="false",
                description="Whether to apply a namespace to the navigation stack",
            ),
            DeclareLaunchArgument(
                name="autostart",
                default_value="true",
                description="Automatically startup the nav2 stack",
            ),
            DeclareLaunchArgument(
                name="map",
                default_value=os.path.join(pkg_share, "maps/test.yaml"),
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                name="params_file",
                default_value=os.path.join(pkg_share, "params/bus_nav2_params.yaml"),
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                name="rviz_config_file",
                default_value=os.path.join(pkg_share, "rviz/nav2_config.rviz"),
                # default_value=os.path.join(nav2_dir, "rviz/nav2_default_view.rviz"),
                description="Full path to the RVIZ config file to use",
            ),
            DeclareLaunchArgument(
                name="slam", default_value="False", description="Whether to run SLAM"
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Whether to start RVIZ",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=[
                    "-d",
                    rviz_config_file,
                    # "--ros-args",
                    # "--log-level",
                    # "debug",
                ],
                output="screen",
                # remappings=[
                #     ("/tf", "tf"),
                #     ("/tf_static", "tf_static"),
                #     ("/goal_pose", "goal_pose"),
                #     ("/clicked_point", "clicked_point"),
                #     ("/initialpose", "initialpose"),
                # ],
            ),
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(nav2_dir, "launch/rviz_launch.py"),
            #     ),
            #     condition=IfCondition(use_rviz),
            #     launch_arguments={
            #         "namespace": namespace,
            #         "rviz_config_file": rviz_config_file,
            #     }.items(),
            # ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_dir, "launch/bringup_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": use_namespace,
                    "slam": slam,
                    "map": map_yaml_file,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": autostart,
                    "log_level": log_level,
                }.items(),
            ),
        ]
    )
