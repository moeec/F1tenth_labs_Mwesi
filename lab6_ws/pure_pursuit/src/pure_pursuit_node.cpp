#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "csv.h"
#include "math.h"

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(x) ((x)/180.0*PI)

const std::string filename = "waypoints.csv";

class PurePursuit : public rclcpp::Node
{
public:
    PurePursuit() : Node("pure_pursuit")
    {
        // Create a subscription to the Odometry message
        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 1000, std::bind(&PurePursuit::callback, this, std::placeholders::_1));


        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 1000);
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1000);

        io::CSVReader<3> in("waypoints.csv");

        double x;
        double y;
        double heading;

        // reading in waypints data from csv
        // based off of https://wiki.ros.org/rviz/DisplayTypes/Marker

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

        while (in.read_row(x, y, heading)) 
        {
            xes.push_back(x);
            yes.push_back(y);
            headings.push_back(heading);
            points.x = x;
            points.y = y;
            points.z = 0.0;
            marker.points.push_back(points);
            //debug below
        }

        marker.lifetime = rclcpp::Duration(0.8);

        marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(),"publish_marker to waypoint: x=%.2f, y=%2f, z=%2f", points.x, points.y, points.z);
   
    }

private:
    // Subscription to Odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;
    double angle;
    double heading_current;
    float steering_angle;
    

    // Callback function for handling Odometry messages
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        // Process the Odometry message

        auto position_odom = msg->pose.pose.position;
        auto orientation_odom = msg->pose.pose.orientation;
       
        double distance_next_x_wp;
        double distance_next_y_wp;
        
        for(int i=0; i < int(xes.size()); i++)
        {
          distance_next_x_wp = xes[i]-position_odom.x;
          distance_next_y_wp = yes[i]-position_odom.y;

          RCLCPP_INFO(this->get_logger(),"Next waypoint: x=%.2f, y=%2f", xes[i], yes[i]);

          steering_angle = DEG2RAD(tan (distance_next_x_wp/distance_next_y_wp));
          drive();
        }
        
        RCLCPP_INFO(this->get_logger(),"nav_msgs:Current Position is: x=%.2f, y=%.2f, z=%.2f", position_odom.x, position_odom.y, position_odom.z);
        RCLCPP_INFO(this->get_logger(),"nav_msgs:Orientation (qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f)", orientation_odom.x, orientation_odom.y, orientation_odom.z, orientation_odom.w);
        RCLCPP_INFO(this->get_logger(),"Next waypoint: x=%.2f, y=%2f", xes[0], yes[0]);
        RCLCPP_INFO(this->get_logger(),"Distance to next waypoint: x=%.2f, y=%2f", distance_next_x_wp, distance_next_y_wp);
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

    void drive() 
    {

        // Create and publish the drive message
        RCLCPP_INFO(this->get_logger(),"drive: Inside drive function");
        RCLCPP_INFO(this->get_logger(),"drive: Steering Angle is: %f", steering_angle);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "/map";

        // Set steering angle and speed
        
        drive_msg.drive.steering_angle = steering_angle;

        // Speed control based on steering angle
        float abs_steering_angle = std::abs(steering_angle);
        if (abs_steering_angle > DEG2RAD(20.0))
        {
            drive_msg.drive.speed = 0.2;
        }
        else if (abs_steering_angle > DEG2RAD(10.0))
        {
            drive_msg.drive.speed = 0.35;
        }
        else
        {
            drive_msg.drive.speed = 0.52;
        }

        ackermann_publisher_->publish(drive_msg);
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}


