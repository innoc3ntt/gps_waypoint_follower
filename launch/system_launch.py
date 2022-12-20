from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("gps_waypoint_follower")
    launch_dir = os.path.join(pkg_share, "launch")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    respawn = LaunchConfiguration("respawn")
    params = LaunchConfiguration("params")
    map_yaml = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "autostart": autostart,
    }

    configured_params = RewrittenYaml(
        source_file=params, param_rewrites=param_substitutions, convert_types=True
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="False",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="True",
                description="Automatically startup gps node",
            ),
            DeclareLaunchArgument(
                "params",
                default_value=os.path.join(pkg_share, "params/ekfs_bus.yaml"),
                description="Parameter file for RL nodes",
            ),
            DeclareLaunchArgument(
                "respawn",
                default_value="True",
                description="Parameter file for RL nodes",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace prefix",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(pkg_share, "maps/test.yaml"),
                description="Map file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "gps_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "respawn": respawn,
                    "params": configured_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "nav2_launch.py")
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "respawn": respawn,
                    "params": configured_params,
                    "map": map_yaml,
                }.items(),
            ),
        ]
    )
