// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"
#include "gps_waypoint_follower.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using AddTwoInts = example_interfaces::srv::AddTwoInts;
using FromLL = robot_localization::srv::FromLL;
auto node = rclcpp::Node::make_shared("minimal_client");
auto from_ll_to_map_client_ = std::make_unique<
    nav2_util::ServiceClient<FromLL>>(
    "/fromLL",
    node);

// std::vector<geographic_msgs::msg::GeoPose> loadGPSWaypointsFromYAML()
geometry_msgs::msg::PoseStamped convertGPS(geographic_msgs::msg::GeoPose gps_pose)
{
  auto req = std::make_shared<FromLL::Request>();
  auto res = std::make_shared<FromLL::Response>();

  geometry_msgs::msg::PoseStamped curr_pose_map_frame;

  req->ll_point.latitude = gps_pose.position.latitude;
  req->ll_point.longitude = gps_pose.position.longitude;
  req->ll_point.altitude = gps_pose.position.altitude;
  from_ll_to_map_client_->wait_for_service((std::chrono::seconds(1)));
  if (!from_ll_to_map_client_->invoke(req, res))
  {
    RCLCPP_ERROR(node->get_logger(), "ll failed");
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Successssss %f", res->map_point.x);

    curr_pose_map_frame.header.frame_id = "map";
    curr_pose_map_frame.header.stamp = node->now();
    curr_pose_map_frame.pose.position = res->map_point;
    curr_pose_map_frame.pose.orientation = gps_pose.orientation;
  }

  return curr_pose_map_frame;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // auto node = std::make_shared<gps_waypoint_folower::GPSWaypointFollower>;
  auto client = node->create_client<AddTwoInts>("add_two_ints");
  auto robo = node->create_client<FromLL>("robo");

  // while (!robo->wait_for_service(std::chrono::seconds(1)))
  // {
  //   if (!rclcpp::ok())
  //   {
  //     RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
  //     return 1;
  //   }
  //   RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  // }

  // while (!client->wait_for_service(std::chrono::seconds(1)))
  // {
  //   if (!rclcpp::ok())
  //   {
  //     RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
  //     return 1;
  //   }
  //   RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
  // }
  geographic_msgs::msg::GeoPose input;

  geometry_msgs::msg::PoseStamped output = convertGPS(input);
  // robo->wait_for_service(std::chrono::seconds(1));

  // auto res_future = robo->async_send_request(req);
  // if (rclcpp::spin_until_future_complete(node, res_future) != rclcpp::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(node->get_logger(), "robo failed you dickhead");
  //   return 1;
  // }
  // auto res = res_future.get();
  // RCLCPP_INFO(node->get_logger(), "result is %f", res->map_point.x);

  // if (!robo->invoke(request, response))

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = 41;
  request->b = 1;
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return 1;
  }
  auto result = result_future.get();
  RCLCPP_INFO(
      node->get_logger(), "result of %" PRId64 " + %" PRId64 " = %" PRId64,
      request->a, request->b, result->sum);
  rclcpp::shutdown();
  return 0;
}
