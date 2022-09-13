#ifndef GPS_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define GPS_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include "geographic_msgs/msg/geo_pose.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace gps_waypoint_follower
{

    using FromLL = robot_localization::srv::FromLL;

    class GPSWaypointFollower : public nav2_util::LifecycleNode
    {
    public:
        // explicit GPSWaypointFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        explicit GPSWaypointFollower();

        ~GPSWaypointFollower();

    protected:
        geometry_msgs::msg::PoseStamped convertGPS(geographic_msgs::msg::GeoPose gps_pose);

        std::unique_ptr<nav2_util::ServiceClient<robot_localization::srv::FromLL>>
            from_ll_to_map_client_;
    };
} // namespace gps_waypoint_foolower

#endif // WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_