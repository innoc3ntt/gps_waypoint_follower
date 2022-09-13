
#include <chrono>
#include <cinttypes>
#include <memory>

// #include "geographic_msgs/msg/geo_pose.hpp"
// #include "nav2_util/service_client.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "robot_localization/srv/from_ll.hpp"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "gps_waypoint_follower.hpp"
namespace gps_waypoint_follower
{
    // auto node = rclcpp::Node::make_shared("gps_waypoint_follower");
    GPSWaypointFollower::GPSWaypointFollower() : nav2_util::LifecycleNode("gps_waypoint_follower", "", false),
                                                 waypoint_task_executor_loader_("nav2_waypoint_follower",
                                                                                "nav2_core::WaypointTaskExecutor")
    {
        RCLCPP_INFO(get_logger(), "Creating");

        declare_parameter("stop_on_failure", true);
        declare_parameter("loop_rate", 20);
        declare_parameter("global_frame_id", global_frame_id_);

        nav2_util::declare_parameter_if_not_declared(
            this, std::string("waypoint_task_executor_plugin"),
            rclcpp::ParameterValue(std::string("wait_at_waypoint")));
        nav2_util::declare_parameter_if_not_declared(
            this, std::string("wait_at_waypoint.plugin"),
            rclcpp::ParameterValue(std::string("nav2_waypoint_follower::WaitAtWaypoint")));
    }

    GPSWaypointFollower::~GPSWaypointFollower()
    {
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "configuring");
        auto node = shared_from_this();

        stop_on_failure_ = get_parameter("stop_on_failure").as_bool();
        loop_rate_ = get_parameter("loop_rate").as_int();
        waypoint_task_executor_id_ = get_parameter("waypoint_task_executor_plugin").as_string();
        global_frame_id_ = get_parameter("global_frame_id").as_string();

        global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

        callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive,
            false);
        callback_group_executor_.add_callback_group(callback_group_, get_node_base_interface());

        nav_to_pose_client_ = rclcpp_action::create_client<ClientT>(
            get_node_base_interface(),
            get_node_graph_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "navigate_to_pose", callback_group_);

        // // TODO: creating service failed, cannot pass lifecycle node to serviceclient
        // // from_ll_to_map_client_ = std::make_unique<nav2_ut
        // from_ll_to_map_client_ = rclcpp_action::create_client<FromLL>(
        //     get_node_base_interface(),
        //     get_node_graph_interface(),
        //     get_node_logging_interface(),
        //     get_node_waitables_interface(),
        //     "fromLL", std::bind(&GPSWaypointFollower::on_activate, this));

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating");
        gps_action_server_->activate();

        createBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        nav_to_pose_client_.reset();
        gps_action_server_.reset();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    GPSWaypointFollower::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");
        gps_action_server_->deactivate();
        destroyBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    geometry_msgs::msg::PoseStamped
    GPSWaypointFollower::convertGPS(geographic_msgs::msg::GeoPose gps_pose)
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
            RCLCPP_ERROR(this->get_logger(), "ll failed");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Successssss %f", res->map_point.x);

            curr_pose_map_frame.header.frame_id = "map";
            curr_pose_map_frame.header.stamp = this->now();
            curr_pose_map_frame.pose.position = res->map_point;
            curr_pose_map_frame.pose.orientation = gps_pose.orientation;
        }

        return curr_pose_map_frame;
    }

    std::vector<geometry_msgs::msg::PoseStamped>
    GPSWaypointFollower::convertGPSPosesToMapPoses(
        const std::vector<geographic_msgs::msg::GeoPose> &gps_poses)
    {
        RCLCPP_INFO(get_logger(), "Converting GPS waypoints to %s Frame..",
                    "map");

        std::vector<geometry_msgs::msg::PoseStamped> poses_in_map_frame_vector;
        int waypoint_index = 0;
        for (auto &&curr_geopose : gps_poses)
        {
            geometry_msgs::msg::PoseStamped curr_pose_map_frame;
            curr_pose_map_frame = GPSWaypointFollower::convertGPS(curr_geopose);
            poses_in_map_frame_vector.push_back(curr_pose_map_frame);
            waypoint_index++;
        }
        return poses_in_map_frame_vector;
    }

    void GPSWaypointFollower::followGPSWaypointsCallback()
    {
        auto goal = gps_action_server_->get_current_goal();
        auto feedback = std::make_shared<ActionTGPS::Feedback>();
        auto result = std::make_shared<ActionTGPS::Result>();

        std::vector<geometry_msgs::msg::PoseStamped> poses;

        poses = convertGPSPosesToMapPoses(
            gps_action_server_->get_current_goal()->gps_poses);

        if (!gps_action_server_ || !gps_action_server_->is_server_active())
        {
            RCLCPP_DEBUG(get_logger(), "Action server inactive. Stopping.");
            return;
        }

        // followWaypointsHandler<std::unique_ptr<ActionServerGPS>,
        //                        ActionTGPS::Feedback::SharedPtr,
        //                        ActionTGPS::Result::SharedPtr>(gps_action_server_,
        //                                                       feedback, result);

        if (poses.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Empty vector of waypoints passed to waypoint following "
                "action potentially due to conversation failure or empty request.");
            gps_action_server_->terminate_current(result);
            return;
        }

        rclcpp::WallRate r(loop_rate_);
        uint32_t goal_index = 0;
        bool new_goal = true;

        while (rclcpp::ok())
        {
            // Check if asked to stop processing action
            if (gps_action_server_->is_cancel_requested())
            {
                auto cancel_future = nav_to_pose_client_->async_cancel_all_goals();
                callback_group_executor_.spin_until_future_complete(cancel_future);
                // for result callback processing
                callback_group_executor_.spin_some();
                gps_action_server_->terminate_all();
                return;
            }

            // Check if asked to process another action
            // if (gps_action_server_->is_preempt_requested())
            // {
            //     RCLCPP_INFO(get_logger(), "Preempting the goal pose.");
            //     goal = gps_action_server_->accept_pending_goal();
            //     poses = getLatestGoalPoses<T>(gps_action_server_);
            //     if (poses.empty())
            //     {
            //         RCLCPP_ERROR(
            //             get_logger(),
            //             "Empty vector of Waypoints passed to waypoint following logic. "
            //             "Nothing to execute, returning with failure!");
            //         gps_action_server_->terminate_current(result);
            //         return;
            //     }
            //     goal_index = 0;
            //     new_goal = true;
            // }

            // Check if we need to send a new goal
            if (new_goal)
            {
                new_goal = false;
                ClientT::Goal client_goal;
                client_goal.pose = poses[goal_index];

                auto send_goal_options = rclcpp_action::Client<ClientT>::SendGoalOptions();
                send_goal_options.result_callback =
                    std::bind(
                        &GPSWaypointFollower::resultCallback,
                        this,
                        std::placeholders::_1);
                send_goal_options.goal_response_callback =
                    std::bind(
                        &GPSWaypointFollower::goalResponseCallback, this,
                        std::placeholders::_1);

                future_goal_handle_ =
                    nav_to_pose_client_->async_send_goal(client_goal, send_goal_options);
                current_goal_status_ = ActionStatus::PROCESSING;
            }

            feedback->current_waypoint = goal_index;
            gps_action_server_->publish_feedback(feedback);

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
                    gps_action_server_->terminate_current(result);
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
                    get_logger(), "Succeeded processing waypoint %i, processing waypoint task execution",
                    goal_index);
                bool is_task_executed = waypoint_task_executor_->processAtWaypoint(
                    poses[goal_index], goal_index);
                RCLCPP_INFO(
                    get_logger(), "Task execution at waypoint %i %s", goal_index,
                    is_task_executed ? "succeeded" : "failed!");
                // if task execution was failed and stop_on_failure_ is on , terminate action
                if (!is_task_executed && stop_on_failure_)
                {
                    failed_ids_.push_back(goal_index);
                    RCLCPP_WARN(
                        get_logger(), "Failed to execute task at waypoint %i "
                                      " stop on failure is enabled."
                                      " Terminating action.",
                        goal_index);
                    result->missed_waypoints = failed_ids_;
                    gps_action_server_->terminate_current(result);
                    failed_ids_.clear();
                    return;
                }
                else
                {
                    RCLCPP_INFO(
                        get_logger(), "Handled task execution on waypoint %i,"
                                      " moving to next.",
                        goal_index);
                }
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
                        get_logger(), "Completed all %zu waypoints requested.",
                        poses.size());
                    result->missed_waypoints = failed_ids_;
                    gps_action_server_->succeeded_current(result);
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

            callback_group_executor_.spin_some();
            r.sleep();
        }
    }
    void GPSWaypointFollower::resultCallback(
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
        const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal)
    {
        if (!goal)
        {
            RCLCPP_ERROR(
                get_logger(),
                "navigate_to_pose action client failed to send goal to server.");
            current_goal_status_ = ActionStatus::FAILED;
        }
    }
}
