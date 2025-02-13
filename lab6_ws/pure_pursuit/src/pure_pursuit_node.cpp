#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "csv.h"
#include "math.h"

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit() : Node("pure_pursuit")
    {
        // Create a subscription to the Odometry message
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1000, std::bind(&PurePursuit::callback, this, std::placeholders::_1));
        
        
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 1000);

        // Other initialization code...

        double x;
        double y;
        double heading;

        // reading in waypints data from csv
        io::CSVReader<3> csv_input("waypoints.csv");

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = this->now();
        marker.ns = "waypoint_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        geometry_msgs::msg::Point points;

        while (csv_input.read_row(x, y, heading)) 
        {
            xes.push_back(x);
            yes.push_back(y);
            headings.push_back(heading);
            points.x = x;
            points.y = y;
            points.z = 0.0;
            marker.points.push_back(points);
        }

        marker.lifetime = rclcpp::Duration(0.1);

        marker_pub_->publish(marker);
    }

private:
    // Subscription to Odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;
    double angle;
    double heading_current;

    // Callback function for handling Odometry messages
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        // Process the Odometry message

        auto position_odom = msg->pose.pose.position;
        auto orientation_odom = msg->pose.pose.orientation;
        
        RCLCPP_INFO(this->get_logger(),"Current Position is: x=%.2f, y=%.2f, z=%.2f", position_odom.x, position_odom.y, position_odom.z);
        RCLCPP_INFO(this->get_logger(),"Orientation (qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f)", orientation_odom.x, orientation_odom.y, orientation_odom.z, orientation_odom.w);

    }

    // Callback for PoseStamped messages
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose data");

        auto position_ps = pose_msg->pose.position;
        auto orientation_ps = pose_msg->pose.orientation;

        RCLCPP_INFO(this->get_logger(),"Current Position is: x=%.2f, y=%.2f, z=%.2f", position_ps.x, position_ps.y, position_ps.z);
        RCLCPP_INFO(this->get_logger(),"Orientation (qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f)", orientation_ps.x, orientation_ps.y, orientation_ps.z, orientation_ps.w);
        
        // Process the PoseStamped message
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
