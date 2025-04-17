#include "spline.h"
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

        tk::spline x_spline, y_spline;
        std::vector<double> s;  // arc-length parameter for interpolation

       // Create uniform parameter "s" based on distance between points
       s.push_back(0.0);
       for (size_t i = 1; i < xes.size(); ++i) 
       {
        double dx = xes[i] - xes[i - 1];
        double dy = yes[i] - yes[i - 1];
        s.push_back(s.back() + std::hypot(dx, dy));
       }

       // Create splines for x(s) and y(s)
       x_spline.set_points(s, xes);
       y_spline.set_points(s, yes);

       // Resample smooth waypoints
       std::vector<double> x_smooth;
       std::vector<double> y_smooth;

      double step = 0.5;  // distance between points
      for (double dist = 0; dist <= s.back(); dist += step) 
      {
        x_smooth.push_back(x_spline(dist));
        y_smooth.push_back(y_spline(dist));
      }

      // Replace xes and yes with smooth ones
      xes = x_smooth;
      yes = y_smooth;
  }

private:
    // Subscription to Odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;

    std::vector<double> xes;
    std::vector<double> yes;
    std::vector<double> headings;
    double distance;
    double prev_distance_x_wp = 100.00;
    double prev_distance_y_wp = 100.00;
    double heading_current;
    float steering_angle;
    float abs_steering_angle;
    int last_good_wp;
    bool lock_steering = false;


    // Callback function for handling Odometry messages
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
//        RCLCPP_INFO(this->get_logger(), "Received odometry data");
        // Process the Odometry message

        auto position_odom = msg->pose.pose.position;
        auto orientation_odom = msg->pose.pose.orientation;

        double distance_next_x_wp;
        double distance_next_y_wp;

        

        
        //RCLCPP_INFO(this->get_logger(), "Odom read %.2f", points.x());

       /*\033[1m = bold
         \033[31m = red   */

        //RCLCPP_INFO(this->get_logger(), "\n\033[1;31mHEADING_CURRENT: --------------------------------------------------<^>------------------------------------- %.2f\033[0m\n", RAD2DEG(heading_current));

        double shortest_distance_to_next_wp = std::numeric_limits<double>::max();

        for(size_t i = 0; i < xes.size(); i++)
        {

             // based off of https://wiki.ros.org/rviz/DisplayTypes/Marker

             visualization_msgs::msg::Marker marker;

             marker.header.frame_id = "/map";
             marker.header.stamp = rclcpp::Clock().now();
 
             marker.ns = "waypoint_marker";
             marker.id = 0;
 
             marker.type = visualization_msgs::msg::Marker::SPHERE;
 
             marker.action = visualization_msgs::msg::Marker::ADD;
 
             marker.pose.position.x = xes[last_good_wp];
             marker.pose.position.y = yes[last_good_wp];
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
 
            distance_next_x_wp = xes[i]-position_odom.x;
            distance_next_y_wp = yes[i]-position_odom.y;


            /*Quaternion to Euler (Yaw) Conversion
            Given (w,x,y,z)
            yaw(heading) = atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) )*/
            double siny_cosp = 2.0 * (orientation_odom.w * orientation_odom.z + orientation_odom.x * orientation_odom.y);
            double cosy_cosp = 1.0 - 2.0 * (orientation_odom.y * orientation_odom.y + orientation_odom.z * orientation_odom.z);
            distance = sqrt((distance_next_x_wp * distance_next_x_wp) + (distance_next_y_wp * distance_next_y_wp));
            heading_current = std::atan2(siny_cosp, cosy_cosp);
            
            // Convert waypoint position to vehicle coordinates
            double dx = xes[i] - position_odom.x;
            double dy = yes[i] - position_odom.y;

           // Rotate waypoint position into vehicle frame
           double local_x =  dx * cos(-heading_current) - dy * sin(-heading_current);
           double local_y =  dx * sin(-heading_current) + dy * cos(-heading_current);

           // Pure Pursuit steering angle calculation is always atan2(local_y, local_x)
           steering_angle = atan2(local_y, local_x);            

           RCLCPP_INFO(this->get_logger(), "\n\033[1;31mHEADING_CURRENT: --------------------------------------------------<^>------------------------------------- %.2f\033[0m\n", RAD2DEG(heading_current));
//            RCLCPP_INFO(this->get_logger(),"Arctancheck: %.2f", RAD2DEG(steering_angle));
//            RCLCPP_INFO(this->get_logger(),"Headingcheck: %.2f", RAD2DEG(heading_current));
           
           
           if(abs(distance) < 0.5 && abs(distance) > 0 && abs(RAD2DEG(steering_angle)) < 180)
           {
             RCLCPP_INFO(this->get_logger(), "\033[1;31m====================== Distance to next x: %.2f(CLOSE) ======================\033[0m", distance_next_x_wp);
             RCLCPP_INFO(this->get_logger(), "\033[1;31m====================== Distance to next y: %.2f(CLOSE) ======================\033[0m", distance_next_y_wp);
             //distance_next_x_wp = xes[i]-position_odom.x;
             //distance_next_y_wp = yes[i]-position_odom.y;
             //double siny_cosp = 2.0 * (orientation_odom.w * orientation_odom.z + orientation_odom.x * orientation_odom.y);
             //double cosy_cosp = 1.0 - 2.0 * (orientation_odom.y * orientation_odom.y + orientation_odom.z * orientation_odom.z);
             //heading_current = std::atan2(siny_cosp, cosy_cosp); 
             last_good_wp = i;
             RCLCPP_INFO(this->get_logger(),"callback: Angle to Waypoint: %.2f", RAD2DEG(steering_angle));
             RCLCPP_INFO(this->get_logger(),"callback: Heading Currently: %.2f", RAD2DEG(heading_current));
             drive();
             stop();
           }
           else
           {
             i++;
           }

           distance = sqrt((distance_next_x_wp * distance_next_x_wp) + (distance_next_y_wp * distance_next_y_wp));
//           RCLCPP_INFO(this->get_logger(),"Distance to next Waypoint: %.2f", distance);

//           RCLCPP_INFO(this->get_logger(),"Next waypoint: x=%.2f, y=%2f", xes[i], yes[i]);
        }


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

//        RCLCPP_INFO(this->get_logger(),"Current Position is: x=%.2f, y=%.2f, z=%.2f", position_ps.x, position_ps.y, position_ps.z);
//        RCLCPP_INFO(this->get_logger(),"Orientation (qx=%.2f, qy=%.2f, qz=%.2f, qw=%.2f)", orientation_ps.x, orientation_ps.y, orientation_ps.z, orientation_ps.w);

        // Process the PoseStamped message
    }

    void drive()
    {

        // Create and publish the drive message

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
       
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "/map";

        // Set steering angle and speed
        
        drive_msg.drive.steering_angle = steering_angle;

        RCLCPP_INFO(this->get_logger(),"drive: Steering Angle is: %f", RAD2DEG(steering_angle));
        RCLCPP_INFO(this->get_logger(),"drive: Heading Currently: %.2f", RAD2DEG(heading_current));

        // Speed control based on steering angle
        abs_steering_angle = std::abs(steering_angle);
        if (abs_steering_angle > DEG2RAD(20.0))
        {
            drive_msg.drive.speed = 0.35;
            RCLCPP_INFO(this->get_logger(),"angle >20 degrees: %f", RAD2DEG(steering_angle));
            RCLCPP_INFO(this->get_logger(),"abs angle >20 degrees: %f", abs_steering_angle);
        }
        else if (abs_steering_angle > DEG2RAD(5.0))
        {
            drive_msg.drive.speed = 0.8;
            RCLCPP_INFO(this->get_logger(),"Speed 0.45");
        }
        else
        {
          drive_msg.drive.speed = 1.0;
        }
        drive_msg.drive.steering_angle = steering_angle;
        ackermann_publisher_->publish(drive_msg);
       // stop();
    }

    void stop()
    {
        // Create and publish the drive message
//        RCLCPP_INFO(this->get_logger(),"STOP: Inside STOP function");
//        RCLCPP_INFO(this->get_logger(),"STOP: Steering Angle is: %f", steering_angle);

        ackermann_msgs::msg::AckermannDriveStamped stop_msg;
       
        stop_msg.header.stamp = this->now();
        stop_msg.header.frame_id = "/map";
        stop_msg.drive.speed = 0.00;
        stop_msg.drive.steering_angle = steering_angle;
        ackermann_publisher_->publish(stop_msg);
    }
    


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}


