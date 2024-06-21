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
        







        

          Publish() 
          {
           //Topics  to publish
           brake_pub = n_.advertise<std_msgs::Bool>("/brake_bool", 1000);
           ack_pub = n_.advertise<ackermann_msgs::AckermannDriveStamped>("/brake", 1000);
          }  


          OdomSubscriber()
          : Node("odom_subscriber")
          {
            sub_ = create_subscription<nav_msgs::msg::Odometry::float64>(
            "/ego_racecar/odom", 36, std::bind(&ScanSubscriber::odom_callback, this, _1));
          }

          ScanSubscriber()
          : Node("scan_subscriber")
          {
            sub_ = create_subscription<sensor_msgs::msg::LaserScan::float32>(
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


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
