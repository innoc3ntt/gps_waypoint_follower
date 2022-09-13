#ifndef GPS_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_
#define GPS_WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_

#include "geographic_msgs/msg/geo_pose.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace gps_waypoint_follower
{
    class GPSWaypointFollower
    {
    public:
        explicit GPSWaypointFollower(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~GPSWaypointFollower();

    protected:
        geometry_msgs::msg::PoseStamped convertGPS(geographic_msgs::msg::GeoPose gps_pose);
    };
} // namespace gps_waypoint_foolower

#endif // WAYPOINT_FOLLOWER__WAYPOINT_FOLLOWER_HPP_