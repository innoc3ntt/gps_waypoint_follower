// Copyright (c) 2019 Samsung Research America
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

#include "gps_waypoint_follower/gps_waypoint_follower.hpp"

#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>

namespace gps_waypoint_follower
{

    GPSWaypointFollower::GPSWaypointFollower()
        : nav2_util::LifecycleNode("GPSWaypointFollower", "", false)
    {
        RCLCPP_INFO(get_logger(), "Creating");

        declare_parameter("stop_on_failure", true);
        declare_parameter("loop_rate", 20);
        declare_parameter("global_frame_id", global_frame_id_);
    }

    GPSWaypointFollower::~GPSWaypointFollower()
    {
        RCLCPP_INFO(get_logger(), "Destroying");
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_configure(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
        loop_rate_ = get_parameter("loop_rate").as_int();
        global_frame_id_ = get_parameter("global_frame_id").as_string();

        std::vector<std::string> new_args = rclcpp::NodeOptions().arguments();
        new_args.push_back("--ros-args");
        new_args.push_back("-r");
        new_args.push_back(std::string("__node:=") + this->get_name() + "_rclcpp_node");
        new_args.push_back("--");
        client_node_ = std::make_shared<rclcpp::Node>(
            "_", "", rclcpp::NodeOptions().arguments(new_args));

        nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
            client_node_, "navigate_to_pose");

        from_ll_to_map_client_ = this->create_client<FromLL>("fromLL");

        action_server_ = std::make_unique<ActionServer>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "FollowGPSWaypoints", std::bind(&GPSWaypointFollower::followWaypoints, this));

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_activate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        action_server_->activate();

        RCLCPP_INFO(get_logger(), "Activate success");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        action_server_->deactivate();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        action_server_.reset();
        nav_to_pose_client_.reset();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    geometry_msgs::msg::PoseStamped
    GPSWaypointFollower::convertGPS(geographic_msgs::msg::GeoPose gps_pose)
    {
        auto req = std::make_shared<FromLL::Request>();

        geometry_msgs::msg::PoseStamped curr_pose_map_frame;

        req->ll_point.latitude = gps_pose.position.latitude;
        req->ll_point.longitude = gps_pose.position.longitude;
        req->ll_point.altitude = gps_pose.position.altitude;

        from_ll_to_map_client_->wait_for_service((std::chrono::seconds(1)));

        auto res = from_ll_to_map_client_->async_send_request(req);
        auto result = res.get();

        RCLCPP_INFO(get_logger(), "sending requiest res %f", res.get()->map_point.x);
        if (from_ll_to_map_client_->service_is_ready())
        {
            RCLCPP_INFO(get_logger(), "sending service request");
            auto res = from_ll_to_map_client_->async_send_request(req);
            RCLCPP_INFO(get_logger(), "service is ready");
            RCLCPP_INFO(get_logger(), "map point y %f", res.get()->map_point.y);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "service not ready ficededadsa");
        }
        curr_pose_map_frame.header.frame_id = "map";
        curr_pose_map_frame.header.stamp = this->now();
        curr_pose_map_frame.pose.position = res.get()->map_point;
        curr_pose_map_frame.pose.orientation = gps_pose.orientation;

        RCLCPP_INFO(get_logger(), "curr pose z%f", curr_pose_map_frame.pose.position.z);

        return curr_pose_map_frame;
    }

    std::vector<geometry_msgs::msg::PoseStamped>
    GPSWaypointFollower::convertGPSPosesToMapPoses(
        const std::vector<geographic_msgs::msg::GeoPose> &gps_poses)
    {
        RCLCPP_INFO(get_logger(), "Converting GPS waypoints to %s Frame..",
                    global_frame_id_.c_str());

        std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
        int waypoint_index = 0;
        for (auto &&curr_geopose : gps_poses)
        {
            RCLCPP_INFO(get_logger(), "Converting GPS waypoints %d [%f,%f] ..",
                        waypoint_index,
                        curr_geopose.position.latitude,
                        curr_geopose.position.longitude);

            geometry_msgs::msg::PoseStamped curr_pose_map_frame;
            curr_pose_map_frame = GPSWaypointFollower::convertGPS(curr_geopose);
            poses_in_map_frame_vector.push_back(curr_pose_map_frame);
            waypoint_index++;
        }
        return poses_in_map_frame_vector;
    }
    void
    GPSWaypointFollower::followWaypoints()
    {
        auto goal = action_server_->get_current_goal();
        auto feedback = std::make_shared<ActionT::Feedback>();
        auto result = std::make_shared<ActionT::Result>();

        std::vector<geometry_msgs::msg::PoseStamped> poses;

        // poses = convertGPSPosesToMapPoses(
        //     action_server_->get_current_goal()->gps_poses);

        // Check if request is valid
        if (!action_server_ || !action_server_->is_server_active())
        {
            RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
            return;
        }

        RCLCPP_INFO(
            get_logger(), "Received follow waypoint request with %i waypoints.",
            static_cast<int>(poses.size()));

        if (poses.size() == 0)
        {
            action_server_->succeeded_current(result);
            return;
        }

        rclcpp::WallRate r(loop_rate_);
        uint32_t goal_index = 0;
        bool new_goal = true;

        while (rclcpp::ok())
        {
            // Check if asked to stop processing action
            if (action_server_->is_cancel_requested())
            {
                auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
                rclcpp::spin_until_future_complete(client_node_, cancel_future);
                // for result callback processing
                spin_some(client_node_);
                action_server_->terminate_all();
                return;
            }

            // Check if asked to process another action
            if (action_server_->is_preempt_requested())
            {
                RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
                goal = action_server_->accept_pending_goal();
                goal_index = 0;
                new_goal = true;
            }

            // Check if we need to send a new goal
            if (new_goal)
            {
                new_goal = false;
                ClientT::Goal client_goal;
                client_goal.pose = poses[goal_index];

                auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
                send_goal_options.result_callback =
                    std::bind(&GPSWaypointFollower::resultCallback, this, std::placeholders::_1);
                send_goal_options.goal_response_callback =
                    std::bind(&GPSWaypointFollower::goalResponseCallback, this, std::placeholders::_1);
                future_goal_handle_ =
                    nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
                current_goal_status_ = ActionStatus::PROCESSING;
            }

            feedback->current_waypoint = goal_index;
            action_server_->publish_feedback(feedback);

            if (current_goal_status_ == ActionStatus::FAILED)
            {
                failed_ids_.push_back(goal_index);

                if (stop_on_failure_)
                {
                    RCLCPP_WARN(
                        get_logger(), "Failed to process waypoint %i in waypoint "
                                      "list and stop on failure is enabled."
                                      " Terminating action.",
                        goal_index);
                    result->missed_waypoints = failed_ids_;
                    action_server_->terminate_current(result);
                    failed_ids_.clear();
                    return;
                }
                else
                {
                    RCLCPP_INFO(
                        get_logger(), "Failed to process waypoint %i,"
                                      " moving to next.",
                        goal_index);
                }
            }
            else if (current_goal_status_ == ActionStatus::SUCCEEDED)
            {
                RCLCPP_INFO(
                    get_logger(), "Succeeded processing waypoint %i, "
                                  "moving to next.",
                    goal_index);
            }

            if (current_goal_status_ != ActionStatus::PROCESSING &&
                current_goal_status_ != ActionStatus::UNKNOWN)
            {
                // Update server state
                goal_index++;
                new_goal = true;
                if (goal_index >= poses.size())
                {
                    RCLCPP_INFO(
                        get_logger(), "Completed all %i waypoints requested.",
                        poses.size());
                    result->missed_waypoints = failed_ids_;
                    action_server_->succeeded_current(result);
                    failed_ids_.clear();
                    return;
                }
            }
            else
            {
                RCLCPP_INFO_EXPRESSION(
                    get_logger(),
                    (static_cast<int>(now().seconds()) % 30 == 0),
                    "Processing waypoint %i...", goal_index);
            }

            rclcpp::spin_some(client_node_);
            r.sleep();
        }
    }

    void
    GPSWaypointFollower::resultCallback(
        const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            current_goal_status_ = ActionStatus::SUCCEEDED;
            return;
        case rclcpp_action::ResultCode::ABORTED:
            current_goal_status_ = ActionStatus::FAILED;
            return;
        case rclcpp_action::ResultCode::CANCELED:
            current_goal_status_ = ActionStatus::FAILED;
            return;
        default:
            current_goal_status_ = ActionStatus::UNKNOWN;
            return;
        }
    }

    void
    GPSWaypointFollower::goalResponseCallback(
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(
                get_logger(),
                "navigate_to_pose action client failed to send goal to server.");
            current_goal_status_ = ActionStatus::FAILED;
        }
    }

} // namespace nav2_waypoint_follower
