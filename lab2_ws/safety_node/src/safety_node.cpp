
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


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

        ros::NodeHandle n;

        {
            brake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/brake_bool", 1000);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&safety_node::timer_callback, this));
        }

        {
            ackerman_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_topic", 10);
            timer_ = this->create_wall_timer(
            500ms, std::bind(&safety_node::timer_callback, this));
        }

        {
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry::float64>(
            "/ego_racecar/odom", 36, std::bind(&ScanSubscriber::odom_callback, this, _1));
        }

        {
            scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan::float32>(
            "scan", 10, std::bind(&ScanSubscriber::scan_callback, this, _1));
        }
        
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        // Extract relative speed (linear velocity x) is only needed as this is straight ahead he
        relative_speed_ = -msg->twist.twist.linear.x

        // Log the velocities
        RCLCPP_INFO(this->get_logger(), "Speed - x: %f ", relative_speed_);
        
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC

        RCLCPP_INFO(this->get_logger(), "Scan: '%f'", msg->data);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan::float32>::SharedPtr subscription_;

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Brake Status is: " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
