#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(x) ((x)/180.0*PI)

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

      //  ackermann_timer_ = this->create_wall_timer(
      //      50ms, std::bind(&WallFollow::ackermann_callback, this)
      //  );

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));


    }

private:
    // PID CONTROL PARAMS
    double kp = 1.00;
    double kd = 0.001; 
    double ki = 0.005;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    
    // Additional Variables 
    rclcpp::TimerBase::SharedPtr ackermann_timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double angle_increment_;
    double angle_min_;
    size_t size; 
    double b_angle;
    double a_angle;
    double alpha_;
    double Dt_;
    double prev_error_ = 0.0;
    rclcpp::Time t_start_time_;
    rclcpp::Time prev_t_start_time_;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    /// TODO: create ROS subscribers and publishers



    double get_range(std::vector<float> range_data, size_t size, double angle)
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

        //RCLCPP_INFO(this->get_logger(), "-----------------------get_range-----------------------------------");
        float range_measurement;
        float returned_range;
        double current_angle_; 

        for (unsigned int i = 0; i < size; i++)
        {
            range_measurement = range_data[i];
            current_angle_ = RAD2DEG(angle_increment_ * i);
            //RCLCPP_INFO(this->get_logger(), "get_range: Current angle is = '%2f'", current_angle_);
            //RCLCPP_INFO(this->get_logger(), "get_range: Input angle is = '%2f'", angle);
            //RCLCPP_INFO(this->get_logger(), "get_range: a_angle = '%2f'", a_angle);
            //RCLCPP_INFO(this->get_logger(), "get_range: a_index = '%2f'", a_index);                              

            if (!std::isinf(range_measurement) && !std::isnan(range_measurement))
            {
                returned_range = range_data[i];
                //RCLCPP_INFO(this->get_logger(), "get_range: inside range measurement w/range = '%2f'", returned_range);
                if (abs(angle - current_angle_) < RAD2DEG(angle_increment_))
                {
                    double diff = abs(angle - current_angle_);
                    //RCLCPP_INFO(this->get_logger(), "Returning Range: abs(angle - current_angle_) = '%2f'", diff);
                    //RCLCPP_INFO(this->get_logger(), "Returning Range: angle_increment_ = '%2f'", angle_increment_);
                    return returned_range;
                }

            }
        }    
        
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

        RCLCPP_INFO(this->get_logger(), "get_error: range_data = '%2f'", range_data);
        RCLCPP_INFO(this->get_logger(), "get_error: dist = '%2f'", dist);
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
        // TODO: Use kp, ki & kd to implement a PID controller
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        // TODO: fill in drive message and publish
        t_start_time_ = this->now();
        rclcpp::Duration delta_t_start_time = t_start_time_ - prev_t_start_time_;
        integral += prev_error * delta_t_start_time.seconds();
        drive_msg.drive.steering_angle = -(kp * error + kd * (error - prev_error) / delta_t_start_time.seconds() + ki * integral);
        prev_t_start_time_ = this->now();

        if (abs(drive_msg.drive.steering_angle) > DEG2RAD(20.0)) 
        {
            drive_msg.drive.speed = 0.5;
            velocity = 0.5;
        } else if (abs(drive_msg.drive.steering_angle) > DEG2RAD(10.0)) 
        {
            drive_msg.drive.speed = 1.0;
            velocity = 0.5;
        } 
        else 
        {
            drive_msg.drive.speed = 1.5;
            velocity = 0.5;
        }


        

        RCLCPP_INFO(this->get_logger(), "pid_control: error = '%2f'", error);
        RCLCPP_INFO(this->get_logger(), "pid_control: velocity = '%2f'", velocity);
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

        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;
        auto range_data_ = scan_msg->ranges;
        size = range_data_.size();

        // b_index = (unsigned int)(floor((DEG2RAD(90) - angle_min_) / angle_increment_));
        b_angle = 90;        // 90.0 / 180.0 * PI; older method
        a_angle = 45;         // 45.0 / 180.0 * PI; older method

        //RCLCPP_INFO(this->get_logger(), "-----------------------scan_callback-----------------------------------");
        //RCLCPP_INFO(this->get_logger(), "scan_callback: b_index before if loop(RAD) = '%2f'", b_index);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_index before if loop(RAD) = '%2f'", a_index);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_angle before if loop(RAD) = '%2f'", a_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_angle before if loop(deg) = '%2f'", RAD2DEG(a_angle));

        /*
        if (angle_min_ > DEG2RAD(45)) 
        {
            a_angle = angle_min_;
            //a_index = 0;
        } 
        else 
        {
            //a_index = (unsigned int)(floor((DEG2RAD(45) - angle_min_) / angle_increment_));
        }
        */

        //RCLCPP_INFO(this->get_logger(), "scan_callback:a_angle(RAD) = '%2f'", a_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback:b_angle(RAD) = '%2f'", b_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback:a_index = '%2f'", a_index);  

        
        /* "\alpha=\mbox{tan}^{-1}\left(\frac{a\mbox{cos}(\theta)-b}{a\mbox{sin}(\theta)}\right)"
        calucation in TeX Commands*/
        
        float a_range = get_range(range_data_, size, a_angle);
        float b_range = get_range(range_data_, size, b_angle);

        double upperValue = (a_range*cos(DEG2RAD(b_angle - a_angle))) - b_range;
        double lowerValue = a_range*sin(DEG2RAD(b_angle - a_angle));

        alpha_ = atan(upperValue/lowerValue); // Calculate the arctangent of the values above
        Dt_ = b_range*cos(alpha_);
        double error = 1 - Dt_; // TODO: replace with error calculated by get_error()

        // double b_range = get_range();

        RCLCPP_INFO(this->get_logger(), "scan_callback: a range returned = '%2f'", a_range);
        RCLCPP_INFO(this->get_logger(), "scan_callback: b range returned = '%2f'", b_range);
        RCLCPP_INFO(this->get_logger(), "scan_callback: Calculated Dt = B - A  = '%2f'", Dt_);
        RCLCPP_INFO(this->get_logger(), "scan_callback: Calculated error = 1 - Dt  = '%2f'", error);

        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

    }



    //unsigned int a_index;
    //unsigned int b_index;

};
int main(int argc, char ** argv) {
    
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}