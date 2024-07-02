#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

/* Notes for timer used on brake 

1. Declaration: rclcpp::Time brake_start_time_; declares a variable named brake_start_time_ that will hold a timestamp.

2. Setting the Timestamp: brake_start_time_ = this->now(); sets brake_start_time_ to the current time when the emergency 
braking is initiated.

3. Elapsed Time Calculation: Later, in the ackermann_callback function, the line (this->now() - brake_start_time_).seconds() calculates the 
elapsed time since the brake_start_time_ was set.*/

class Safety : public rclcpp::Node 
{
public:
    Safety() : Node("safety_node"), brakenow_(false), braking_(false)
    {
        brake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/brake_bool", 1000);
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        ackermann_timer_ = this->create_wall_timer(
            50ms, std::bind(&Safety::ackermann_callback, this)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 36, std::bind(&Safety::odom_callback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
    }

private:
    void ackermann_callback()
    {
        auto message = ackermann_msgs::msg::AckermannDriveStamped();

        if (braking_)
        {
            message.drive.speed = 0.0;
            RCLCPP_INFO(this->get_logger(), "Braking...");

            // Publish the brake command
            brake_publisher_->publish(std_msgs::msg::Bool().set__data(true));

            // Check if the braking duration has passed
            if ((this->now() - brake_start_time_).seconds() >= 5.0)
            {
                braking_ = false;
                message.drive.speed = 0.0;
                RCLCPP_INFO(this->get_logger(), "Resuming normal operation.");
            }
            else
            {
                // Set speed to 0 or previous speed before braking
                message.drive.speed = 0.0; // or set to the last known safe speed
                brake_publisher_->publish(std_msgs::msg::Bool().set__data(false));
            }
        }

        // Log the velocities
        RCLCPP_INFO(this->get_logger(), "Ackermann - Speed Input: %f", message.drive.speed);

        // Publish the message
        ackermann_publisher_->publish(message);
    }                   
    
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        relative_speed_ = -msg->twist.twist.linear.x;
        v_x = msg->twist.twist.linear.x;
        v_y = msg->twist.twist.linear.y;
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        auto range_measured = scan_msg->ranges;
        auto ackermann_drive = ackermann_msgs::msg::AckermannDriveStamped();

        for (unsigned int i = 0; i < range_measured.size(); i++)
        {
            distance_ = scan_msg->ranges[i];
            angle_increment_ = scan_msg->angle_increment;
            angle_ = scan_msg->angle_min;
            current_angle_ = angle_ + angle_increment_ * i;
            range_rate_ = cos(current_angle_) * v_x + sin(current_angle_) * v_y;                               

            if (!std::isinf(distance_) && !std::isnan(distance_))
            {
                if (range_rate_ > 0)    
                {
                    min_TTC = distance_ / range_rate_;
                    if (min_TTC <= TTC_threshold) 
                    {
                        RCLCPP_INFO(this->get_logger(), "Automatic Emergency Braking Activated TTC = '%f'", min_TTC);
                        brakenow_ = true;
                        braking_ = true;
                        brake_start_time_ = this->now();
                        ackermann_drive.drive.speed = 0.0;
                    }
                }
                else
                {
                    brakenow_ = false;
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Scan Distance is inf or NaN");
            }
        }
    }

    rclcpp::TimerBase::SharedPtr ackermann_timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    double TTC_threshold = 1.0;
    double min_TTC = std::numeric_limits<double>::max(); 
    double relative_speed_ = 0.0;
    double range_rate_;
    bool brakenow_;
    bool braking_;
    double distance_;
    double angle_;
    double angle_increment_;
    double current_angle_;
    double v_x;
    double v_y;
    rclcpp::Time brake_start_time_;
};

int main(int argc, char ** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}

