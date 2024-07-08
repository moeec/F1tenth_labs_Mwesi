#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define PI 3.1415927

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        ackermann_timer_ = this->create_wall_timer(
            50ms, std::bind(&WallFollow::ackermann_callback, this)
        );

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));


    }

private:
    // PID CONTROL PARAMS
    double kp = 1.00
    double kd = 0.001 
    double ki = 0.005
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers



    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // TODO: implement


        auto scan = sensor_msgs::msg::LaserScan();

        angle_increment_ = scan.angle_increment;
        angle_min_ = scan.angle_min;
        angle_max_ = scan.angle_max;
        double current_angle_;
        

        double closest = std::numeric_limits<double>::max();
        double minDiff = std::numeric_limits<double>::max();
        
        for (unsigned int i = 0; i < range_data.size(); i++) 
        {
            current_angle_ = angle_min + angle_increment_ * i;
            double diff = std::abs(current_angle_ - angle);
            
            if (diff < minDiff) 
            {
            minDiff = diff;
            closest = current_angle_;
            }
        }

        std::cout << "Closest value to " << angle << " is " << closest << std::endl;


        

        b_angle = PI/180; 
        double angle_increment = angle/range_data.size();
        double range_measurement;

        for (unsigned int i = 0; i < range_data.size(); i++)
        {
            range_measurement = range_data[i];
            current_angle_ = angle_increment * i;                             

            if (!std::isinf(range_measurement) && !std::isnan(range_measurement))
            {


            }
        }


        range_to_wall = 
          
        return 0.0;
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        // TODO:implement
        return 0.0;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        auto range_array = scan_msg->ranges;
        auto laser_scan = scan_msg;
        auto ackermann_drive = ackermann_msgs::msg::AckermannDriveStamped();
        auto angle_min_ = scan_msg->angle_min
        auto angle_max_ = scan_msg->angle_max

        double get_range(range_array,angle_);
















        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

    }


    double findClosestNumber(const std::vector<double>& arr, double target) 
    {

        /*The findClosestNumber function takes a std::vector<double> and a double target as input.
          It initializes closest and minDiff with maximum possible values.
          It iterates through each number in the array, calculates the absolute difference between 
          the current number and the target, and updates closest if this difference is smaller than the current minDiff.
          Finally, the function returns the closest number.*/


        double closest = std::numeric_limits<double>::max();
        double minDiff = std::numeric_limits<double>::max();
        
        for (const double& num : arr) 
        {
            double diff = std::abs(num - target);
            
            if (diff < minDiff) 
            {
            minDiff = diff;
            closest = num;
            }
        }

    return closest;
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}