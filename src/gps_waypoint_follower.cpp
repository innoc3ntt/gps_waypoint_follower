
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
    GPSWaypointFollower::GPSWaypointFollower(const rclcpp::NodeOptions &options)
    {

        auto node = shared_from_this();
    }

    GPSWaypointFollower::~GPSWaypointFollower()
    {
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

}