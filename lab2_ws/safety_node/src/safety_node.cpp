
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float64.h>

using namespace std::chrono_literals;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {

        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers

        //ros::NodeHandle n;

        {
            brake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/brake_bool", 1000);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&Safety::brake_callback, this));
        }
        
        {
            ackerman_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&Safety::ackerman_callback, this));
        }
        
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry::float64>(
            "/ego_racecar/odom", 36, std::bind(&Safety::odom_callback, this, _1));
        }
        
        {
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan::float32>(
            "scan", 10, std::bind(&Safety::scan_callback, this, _1));
        }
        
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    /*
    void brake_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        /// Update brake status
        RCLCPP_INFO(this->get_logger(), "Brake Status is - %s", brake_publisher_ ? "true" : "false");    
    }
    rclcpp::Publisher<std_msgs::msg::Bool::SharedPtr brake_publisher_ ; */ 
    
    void brake_callback()
    {
        // Create a message of type std_msgs::msg::Bool
        auto message = std_msgs::msg::Bool();
        
        // Set the data field to a boolean value
        message.data = false;  // Assuming starting with false boolean value
        
        // Log the message with the correct format
        RCLCPP_INFO(rclcpp::get_logger("brake_callback"), "Publishing: 'Brake Status is %s'", message.data ? "true" : "false");
        
        // Publish the message
        publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_publisher_;
                     
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        // Extract relative speed (linear velocity x) is only needed as this is straight ahead he
        relative_speed_ = -msg->twist.twist.linear.x;

        // Log the velocities
        RCLCPP_INFO(this->get_logger(), "Speed - x: %f",relative_speed_);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC

        RCLCPP_INFO(this->get_logger(), "Scan: '%f'", msg->data);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan::float32>::SharedPtr scan_sub_;

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
