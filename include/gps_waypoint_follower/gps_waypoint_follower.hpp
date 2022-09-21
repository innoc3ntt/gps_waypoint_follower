#ifndef GPS_WAYPOINT_FOLLOWER_HPP_
#define GPS_WAYPOINT_FOLLOWER_HPP_

#include <cinttypes>
#include <memory>

#include "geographic_msgs/msg/geo_pose.hpp"
#include "gps_interfaces/action/follow_gps_waypoints.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/service_client.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/string_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace gps_waypoint_follower
{
    enum class ActionStatus
    {
        UNKNOWN = 0,
        PROCESSING = 1,
        FAILED = 2,
        SUCCEEDED = 3
    };

    /**
     * @class nav2_gps_waypoint_follower::GPSWaypointFollower
     * @brief An action server that uses behavior tree for navigating a robot to its
     * goal position.
     */
    class GPSWaypointFollower : public nav2_util::LifecycleNode
    {
    public:
        using ActionT = nav2_msgs::action::FollowWaypoints;
        using ClientT = nav2_msgs::action::NavigateToPose;
        using ActionServer = nav2_util::SimpleActionServer<ActionT>;
        using ActionClient = rclcpp_action::Client<ClientT>;
        using FromLL = robot_localization::srv::FromLL;

        using ActionTGPS = gps_interfaces::action::FollowGPSWaypoints;
        using ActionServerGPS = nav2_util::SimpleActionServer<ActionTGPS>;

        /**
         * @brief A constructor for nav2_gps_waypoint_follower::GPSWaypointFollower class
         * @param options Additional options to control creation of the node.
         */
        explicit GPSWaypointFollower();

        // Destructor
        ~GPSWaypointFollower();

    protected:
        // override callbacks for lifecycle node
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        //servers and clients
        rclcpp::Client<FromLL>::SharedPtr from_ll_to_map_client_;
        std::unique_ptr<ActionServerGPS> gps_action_server_;
        ActionClient::SharedPtr nav_to_pose_client_;

        rclcpp::CallbackGroup::SharedPtr callback_group_;
        rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

        // Instance functions
        geometry_msgs::msg::PoseStamped convertGPS(geographic_msgs::msg::GeoPose gps_pose);
        void followGPSWaypointsCallback();
        void resultCallback(const rclcpp_action::ClientGoalHandle<ClientT>::WrappedResult &result);
        void goalResponseCallback(const rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr &goal);
        std::vector<geometry_msgs::msg::PoseStamped>
        convertGPSPosesToMapPoses(const std::vector<geographic_msgs::msg::GeoPose> &gps_poses);

        // internal variables
        std::shared_future<rclcpp_action::ClientGoalHandle<ClientT>::SharedPtr> future_goal_handle_;
        ActionStatus current_goal_status_;
        std::vector<int> failed_ids_;

        // params
        int loop_rate_;
        bool stop_on_failure_;
        std::string global_frame_id_;

        // waypoint plugins
        pluginlib::ClassLoader<nav2_core::WaypointTaskExecutor>
            waypoint_task_executor_loader_;
        pluginlib::UniquePtr<nav2_core::WaypointTaskExecutor>
            waypoint_task_executor_;
        std::string waypoint_task_executor_id_;
        std::string waypoint_task_executor_type_;
    };
} // namespace gps_waypoint_follower

#endif //GPS_WAYPOINT_FOLLOWER_HPP_