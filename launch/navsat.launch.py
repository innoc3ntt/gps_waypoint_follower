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
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("gps_waypoint_follower")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params")

    param_substitutions = {
        "use_sim_time": use_sim_time,
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
                "params",
                default_value=os.path.join(pkg_share, "params/ekfs_demo.yaml"),
                description="",
            ),
            Node(
                package="robot_localization",
                condition=IfCondition(PythonExpression(["not ", use_sim_time])),
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[configured_params],
                remappings=[
                    ("imu", "imu/data"),
                    ("gps/fix", "imu/nav_sat_fix"),  # ! from sbg
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                condition=IfCondition(use_sim_time),
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[configured_params],
                remappings=[
                    ("imu", "imu/data"),
                    ("gps/fix", "gps/fix"),  # ! for demo
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
        ]
    )