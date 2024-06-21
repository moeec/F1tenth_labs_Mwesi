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

        public:
          OdomSubscriber()
          : Node("odom_subscriber")
          {
            sub_ = create_subscription<nav_msgs::msg::Odometry::String>(
            "/ego_racecar/odom", 10, std::bind(&ScanSubscriber::odom_callback, this, _1));
          }

        public:
          ScanSubscriber()
          : Node("scan_subscriber")
          {
            sub_ = create_subscription<sensor_msgs/LaserScan::msg::float32>(
            "scan", 10, std::bind(&ScanSubscriber::scan_callback, this, _1));
          }
        
    }

private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed

        // Extract linear velocity
        double linear_velocity_x = msg->twist.twist.linear.x;
        double linear_velocity_y = msg->twist.twist.linear.y;
        double linear_velocity_z = msg->twist.twist.linear.z;

        // Extract angular velocity
        double angular_velocity_x = msg->twist.twist.angular.x;
        double angular_velocity_y = msg->twist.twist.angular.y;
        double angular_velocity_z = msg->twist.twist.angular.z;

        // Log the velocities
        RCLCPP_INFO(this->get_logger(), "Linear Velocity - x: %f, y: %f, z: %f", linear_velocity_x, linear_velocity_y, linear_velocity_z);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity - x: %f, y: %f, z: %f", angular_velocity_x, angular_velocity_y, angular_velocity_z);
    }
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC

        RCLCPP_INFO(this->get_logger(), "Scan: '%f'", msg->data);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan::float32>::SharedPtr subscription_;



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
