#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <limits>
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

        std::ifstream file_check("waypoints.csv");
        if (!file_check.is_open())
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints.csv");
          return;
        }
        file_check.close();


        io::CSVReader<3> in("waypoints.csv");
        double x;
        double y;
        double heading;

        // reading in waypints data from csv

        while (in.read_row(x, y, heading)) 
        {
            xes.push_back(x);
            yes.push_back(y);
            headings.push_back(heading);
        }   
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
    double distance;
    double prev_distance_x_wp = 100.00;
    double prev_distance_y_wp = 100.00;
    double heading_current;
    float steering_angle;
    int matrix;
    

    // Callback function for handling Odometry messages
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        // Process the Odometry message

        auto position_odom = msg->pose.pose.position;
        auto orientation_odom = msg->pose.pose.orientation;
       
        double distance_next_x_wp;
        double distance_next_y_wp;

        /*Quaternion to Euler (Yaw) Conversion
        Given (w,x,y,z)
        yaw(heading) = atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) )*/ 

        double siny_cosp = 2.0 * (orientation_odom.w * orientation_odom.z + orientation_odom.x * orientation_odom.y);
        double cosy_cosp = 1.0 - 2.0 * (orientation_odom.y * orientation_odom.y + orientation_odom.z * orientation_odom.z);
        auto heading_current = std::atan2(siny_cosp, cosy_cosp);

        //RCLCPP_INFO(this->get_logger(), "Odom read %.2f", points.x());

        RCLCPP_INFO(this->get_logger(),"nav_msgs:Heading_current:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- %.2f", RAD2DEG(heading_current));

        double shortest_distance_to_next_wp = std::numeric_limits<double>::max();
        
        for(size_t i = 0; i < xes.size(); i++)
        {
            distance_next_x_wp = xes[0]-position_odom.x;
            distance_next_y_wp = yes[0]-position_odom.y;
            matrix = static_cast<int>(i);
           
           if(abs(distance_next_x_wp) < 0.4 || abs(prev_distance_x_wp) < 0.4) 
           {
             RCLCPP_INFO(this->get_logger(),"Distance to next x: %.2f", distance_next_x_wp);
           }
           else
           {
             RCLCPP_INFO(this->get_logger(),"Far from next x, correcting: %.2f", distance_next_x_wp);
             if(prev_distance_x_wp > distance_next_x_wp)
             {
               prev_distance_x_wp = distance_next_x_wp;
             } 
           }

           if(abs(distance_next_y_wp) < 1.0) 
           {
             RCLCPP_INFO(this->get_logger(),"Distance to next y: %.2f", distance_next_y_wp); 
           }
           else
           {
             RCLCPP_INFO(this->get_logger(),"Far from next y, correcting: %.2f", distance_next_y_wp);
             if(prev_distance_y_wp > distance_next_y_wp)
             {
               prev_distance_y_wp = distance_next_y_wp;
             }
           }

            angle = std::atan2(distance_next_y_wp, distance_next_x_wp);
            distance = sqrt((distance_next_x_wp * distance_next_x_wp) + (distance_next_y_wp * distance_next_y_wp));
            RCLCPP_INFO(this->get_logger(),"Angle to Waypoint: %.2f", RAD2DEG(angle));
            RCLCPP_INFO(this->get_logger(),"Distance to next Waypoint: %.2f", distance);

            //RCLCPP_INFO(this->get_logger(),"nav_msgs:Angle: %.2f",RAD2DEG(angle));

            //RCLCPP_INFO(this->get_logger(),"nav_msgs:Distance: %.2f", distance);
            
            if(distance < shortest_distance_to_next_wp)
            {
                shortest_distance_to_next_wp = distance; 
                RCLCPP_INFO(this->get_logger(),"nav_msgs: Shortest distance: %.2f", distance);
            }
          

            RCLCPP_INFO(this->get_logger(),"Next waypoint: x=%.2f, y=%2f", xes[i], yes[i]);

            steering_angle = angle;   
        }

        drive();

        // based off of https://wiki.ros.org/rviz/DisplayTypes/Marker
        
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = "/map";
        marker.header.stamp = rclcpp::Clock().now();

        marker.ns = "waypoint_marker";
        marker.id = 0;

        marker.type = visualization_msgs::msg::Marker::SPHERE;

        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = xes[0];
        marker.pose.position.y = yes[0];
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;   // Don't forget to set the alpha!

        marker.lifetime = rclcpp::Duration(0,0);

        marker_pub_->publish(marker);

        
        //RCLCPP_INFO(this->get_logger(),"nav_msgs:Orientation (qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f)", orientation_odom.x, orientation_odom.y, orientation_odom.z, orientation_odom.w);
        //RCLCPP_INFO(this->get_logger(),"Distance to next waypoint: x=%.2f, y=%2f", distance_next_x_wp, distance_next_y_wp);
    }

    // Callback for PoseStamped messages
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received pose data");

        auto position_ps = pose_msg->pose.position;
        auto orientation_ps = pose_msg->pose.orientation;

        
        /*Quaternion to Euler (Yaw) Conversion
        Given (w,x,y,z)
        yaw(heading) = atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) )*/ 
        
        
        double siny_cosp = 2.0 * (orientation_ps.w * orientation_ps.z + orientation_ps.x * orientation_ps.y);
        double cosy_cosp = 1.0 - 2.0 * (orientation_ps.y * orientation_ps.y + orientation_ps.z * orientation_ps.z);
        auto heading_current = std::atan2(siny_cosp, cosy_cosp);

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
            RCLCPP_INFO(this->get_logger(),"Speed 0.35");
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


