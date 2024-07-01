
#include <chrono>
#include <string>
#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class Safety : public rclcpp::Node 
{
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
        
        brake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/brake_bool", 1000);
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_topic", 10);

        /*brake_timer_ = this->create_wall_timer(
            500ms, std::bind(&Safety::brake_callback, this)
        );*/

        ackermann_timer_ = this->create_wall_timer(
            500ms, std::bind(&Safety::ackermann_callback, this)
        );
        
        {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 36, std::bind(&Safety::odom_callback, this, std::placeholders::_1));
        }
        
        {
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Safety::scan_callback, this, std::placeholders::_1));
        }
        
    }

private:

    void brake_callback()
    {
        // Create a message of type std_msgs::msg::Bool
        auto message = std_msgs::msg::Bool();
        
        // Set the data field to a boolean value
        message.data = false;  // Assuming starting with false boolean value  

        if(brakenow_)
        {
            message.data = true;
            RCLCPP_INFO(rclcpp::get_logger("brake_callback"), "Publishing: 'Brake Status is %s'", message.data ? "true" : "false");
        }
        else
        {
            // Log the message with the correct format
            RCLCPP_INFO(rclcpp::get_logger("brake_callback"), "Publishing: 'Brake Status is %s'", message.data ? "true" : "false");
        }
        
        // Publish the message
        brake_publisher_->publish(message);
    } 
    
    void ackermann_callback()
    {

        auto message = ackermann_msgs::msg::AckermannDriveStamped();

        if(brakenow_)
        {
            message.drive.speed = 0.0;

            RCLCPP_INFO(this->get_logger(), "Too Close Brake Event");

            // Brake for 3 seconds before rechecking
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }

        // Log the velocities
        RCLCPP_INFO(this->get_logger(), "Ackermann - Speed Input : %f",message.drive.speed);

        // Publish the message
        ackermann_publisher_->publish(message);

    }                   
    
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        // Extract relative speed (linear velocity x) is only needed as this is straight ahead he
        relative_speed_ = -msg->twist.twist.linear.x;
        v_x = msg->twist.twist.linear.x;
        v_y = msg->twist.twist.linear.y;

        // Log the velocities
        //RCLCPP_INFO(this->get_logger(), "Odom - Speed - x: %f",relative_speed_);
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// Calculate instantaneous Time-To-Collision. 
        /// iTTC=\frac{r}{\lbrace- \dot{r}\rbrace_{+}} 

        auto range_measured = scan_msg->sensor_msgs::msg::LaserScan::ranges;
        auto ackermann_drive = ackermann_msgs::msg::AckermannDriveStamped();

        for (unsigned int i = 0; i < range_measured.size(); i++)
        {
            // instantaneous range measurement(r).
            distance_ = scan_msg->sensor_msgs::msg::LaserScan::ranges[i];

            angle_increment_ = scan_msg->sensor_msgs::msg::LaserScan::angle_increment;
            angle_ = scan_msg->sensor_msgs::msg::LaserScan::angle_min;
            
            // Angle from X that is straight forward to ray projection 
            current_angle_= angle_ + angle_increment_ * i;
            current_angle_degrees_ = current_angle_ * (180/3.14159265359);
            
            // Range rate (dot{r}), indicates how fast the distance is changing (derivative of r)
            range_rate_ = cos(current_angle_) * v_x + sin(current_angle_) * v_y;                               

            if (!std::isinf(distance_) && !std::isnan(distance_))
            {
              // RCLCPP_INFO(this->get_logger(), "Scan Distance is: '%f'", distance_); // for debugging
              // RCLCPP_INFO(this->get_logger(), "Scan Angle(radians) is: '%f'", current_angle_); // for debugging
              // RCLCPP_INFO(this->get_logger(), "Scan Angle(degrees) is: '%f'", current_angle_degrees_); // for debugging
              // RCLCPP_INFO(this->get_logger(), "Range Rate is: '%f'", range_rate_); // for debugging 

               // If range rate((dot{r})) is greater than 0, keep (dot{r}) . If (dot{r}) is less than 0, 0 is used.
               if (range_rate_ > 0)    
               {
                   min_TTC = distance_ / range_rate_;
                   //RCLCPP_INFO(this->get_logger(), "Minimum Time to Collision is: '%f'", min_TTC); // for debug
                   if (min_TTC <= TTC_threshold) 
                   {
                       // Brake Event here
                       RCLCPP_INFO(this->get_logger(), "Automatic Emergency Braking Activated TTC = '%f'", min_TTC);
                       brakenow_ = true;
                       

                        for (int j = 0; j < 1000; j++)
                        {
                            ackermann_drive.drive.speed = 0.0;
                            std::this_thread::sleep_for(std::chrono::milliseconds(3)); // Sleep for a short duration
                        }
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

    rclcpp::TimerBase::SharedPtr brake_timer_;
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
    double distance_;
    double angle_;
    double angle_increment_;
    double current_angle_;
    double current_angle_degrees_;
    double v_x;
    double v_y;
    
};

int main(int argc, char ** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}






