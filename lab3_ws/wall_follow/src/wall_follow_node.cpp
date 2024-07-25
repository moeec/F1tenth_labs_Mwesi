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
#define DESIRED_DISTANCE_RIGHT 0.4

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

    }

private:
    // PID CONTROL PARAMS
    double kp = 0.250;
    double kd = 6.550; 
    double ki = 0.010;
    double servo_offset = 0.0;
    double error = 0.0;
    double integral = 0.0;
    
    // Additional Variables 
    double angle_increment_;
    double angle_min_;
    size_t size; 
    double b_angle;
    double a_angle;
    double alpha_;
    double Dt_;
    double Dt_t1_;
    double prev_error_ = 0.0;
    rclcpp::Time t_start_time_;
    rclcpp::Time prev_t_start_time_;

    // Publisher & Subscribers 
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

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

        //RCLCPP_INFO(this->get_logger(), "-----------------------get_range-----------------------------------"); //for debugging
        float range_measurement;
        float returned_range;
        double current_angle_; 

        for (unsigned int i = 0; i < size; i++)
        {
            range_measurement = range_data[i];
            current_angle_ = RAD2DEG(angle_increment_ * i);
            //RCLCPP_INFO(this->get_logger(), "get_range: Current angle is = '%2f'", current_angle_); //for debugging
            //RCLCPP_INFO(this->get_logger(), "get_range: Input angle is = '%2f'", angle);            //for debugging
            //RCLCPP_INFO(this->get_logger(), "get_range: a_angle = '%2f'", a_angle);                 //for debugging
            //RCLCPP_INFO(this->get_logger(), "get_range: a_index = '%2f'", a_index);                 //for debugging                             

            if (!std::isinf(range_measurement) && !std::isnan(range_measurement))
            {
                returned_range = range_data[i];
                //RCLCPP_INFO(this->get_logger(), "get_range: inside range measurement w/range = '%2f'", returned_range); //for debugging
                if (abs(angle - current_angle_) < RAD2DEG(angle_increment_))
                {
                    //double diff = abs(angle - current_angle_); /for debugging
                    //RCLCPP_INFO(this->get_logger(), "Returning Range: abs(angle - current_angle_) = '%2f'", diff);     //for debugging
                    //RCLCPP_INFO(this->get_logger(), "Returning Range: angle_increment_ = '%2f'", angle_increment_);    //for debugging
                    //RCLCPP_INFO(this->get_logger(), "get_range: angle = '%2f'", current_angle_ );    //for debugging
                    return returned_range;
                }

            }
        }
        return 0.0;     
    }

    double get_error(double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double Dtp1_= DESIRED_DISTANCE_RIGHT - dist;
        RCLCPP_INFO(this->get_logger(), "get_error: dist = '%2f'", dist);
        RCLCPP_INFO(this->get_logger(), "get_error: 1 - dist = '%2f'", Dtp1_);
        
        return Dtp1_;
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

        RCLCPP_INFO(this->get_logger(), "-----------------------pid_control-----------------------------------"); //for debugging
        
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
      
        t_start_time_ = this->now();
        double delta_t_start_time = t_start_time_.seconds() - prev_t_start_time_.seconds();
        RCLCPP_INFO(this->get_logger(), "pid_control: prev_error_ (before) = '%2f'", prev_error_);
        RCLCPP_INFO(this->get_logger(), "pid_control: error_ (coming in) = '%2f'", error);

        // Proportional term
        double P = kp * error;

        // Integral term
        integral += prev_error_ * delta_t_start_time;
        RCLCPP_INFO(this->get_logger(), "pid_control: error_ (coming in) = '%2f'", error);
        RCLCPP_INFO(this->get_logger(), "pid_control: integral = '%2f'", integral);
        RCLCPP_INFO(this->get_logger(), "pid_control: delta_t_start_time = '%2f'", delta_t_start_time);
        double I = ki * integral;

        // Derivative term
        double derivative = (error - prev_error_) / delta_t_start_time;
        double D = kd * derivative;
        
        /*
        Older equation kept for archiving and comparison 
        drive_msg.drive.steering_angle = -(kp * error + kd * (error - prev_error_) / delta_t_start_time + ki * integral);
        */

        drive_msg.drive.steering_angle = (P + I + D);
        
        prev_error_ = error;
        prev_t_start_time_ = t_start_time_;
        RCLCPP_INFO(this->get_logger(), "pid_control: prev_error_ (after) = '%2f'", prev_error_);
        RCLCPP_INFO(this->get_logger(), "pid_control: t_start_time_ = '%2f'", t_start_time_);
        RCLCPP_INFO(this->get_logger(), "pid_control: P = '%2f'", P); 
        RCLCPP_INFO(this->get_logger(), "pid_control: I = '%2f'", I); 
        RCLCPP_INFO(this->get_logger(), "pid_control: D = '%2f'", D);  

        RCLCPP_INFO(this->get_logger(), "pid_control: drive_msg.drive.steering_angle = '%2f'", RAD2DEG(drive_msg.drive.steering_angle));
        
        if (abs(drive_msg.drive.steering_angle) > DEG2RAD(20.0)) 
        {
            drive_msg.drive.speed = 0.5;
            velocity = 0.5;
            RCLCPP_INFO(this->get_logger(), "---------------------Drive Speed = 0.5;---------------------------------"); //for debugging
        } 
        else if (abs(drive_msg.drive.steering_angle) > DEG2RAD(10.0)) 
        {
            drive_msg.drive.speed = 1.0;
            velocity = 1.0;
            RCLCPP_INFO(this->get_logger(), "---------------------Drive Speed = 1.0;---------------------------------"); //for debugging
        } 
        else 
        {
            drive_msg.drive.speed = 1.5;
            velocity = 1.5;
            RCLCPP_INFO(this->get_logger(), "---------------------Drive Speed = 1.5;---------------------------------"); //for debugging
        }

        ackermann_publisher_->publish(drive_msg);

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

        auto range_data_ = scan_msg->ranges;
       
        b_angle = 90;        
        a_angle = 60.629;         

        //RCLCPP_INFO(this->get_logger(), "-----------------------scan_callback-----------------------------------");
        //RCLCPP_INFO(this->get_logger(), "scan_callback: b_index before if loop(RAD) = '%2f'", b_index);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_index before if loop(RAD) = '%2f'", a_index);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_angle before if loop(RAD) = '%2f'", a_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback: a_angle before if loop(deg) = '%2f'", RAD2DEG(a_angle));
        //RCLCPP_INFO(this->get_logger(), "scan_callback:a_angle(RAD) = '%2f'", a_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback:b_angle(RAD) = '%2f'", b_angle);
        //RCLCPP_INFO(this->get_logger(), "scan_callback:a_index = '%2f'", a_index);  

        /* "\alpha=\mbox{tan}^{-1}\left(\frac{a\mbox{cos}(\theta)-b}{a\mbox{sin}(\theta)}\right)"
        calucation in TeX Commands*/

        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;
        size = range_data_.size();
        float a_range = get_range(range_data_, size, a_angle);


        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;
        size = range_data_.size();
        float b_range = get_range(range_data_, size, b_angle);

        double upperValue = (a_range*cos(DEG2RAD(b_angle - a_angle))) - b_range;
        double lowerValue = a_range*sin(DEG2RAD(b_angle - a_angle));

        alpha_ = atan(upperValue/lowerValue); // Calculate the arctangent of the values above
        
        Dt_ = b_range*cos(alpha_);
        Dt_t1_ = Dt_ + 1.00*sin(alpha_);

        // Calculate error with lookahead distance
        double error = get_error(Dt_t1_);
        
        double velocity = 1.5; // TODO: calculate desired car velocity based on error

        RCLCPP_INFO(this->get_logger(), "scan_callback: a angle returned = '%2f'", a_angle);
        RCLCPP_INFO(this->get_logger(), "scan_callback: b angle returned = '%2f'", b_angle);
        RCLCPP_INFO(this->get_logger(), "scan_callback: a range returned = '%2f'", a_range);
        RCLCPP_INFO(this->get_logger(), "scan_callback: b range returned = '%2f'", b_range);
        RCLCPP_INFO(this->get_logger(), "scan_callback: Calculated Dt = B - A  = '%2f'", Dt_);
        RCLCPP_INFO(this->get_logger(), "scan_callback: Calculated Dt_t1_ = Dt_ + 1.00*sin(alpha_);  = '%2f'", Dt_t1_);
        RCLCPP_INFO(this->get_logger(), "scan_callback: Calculated error = 1 - Dt_t1  = '%2f'", error);

        // actuate the car with PID  
        pid_control(error, velocity); 
        
    }

};
int main(int argc, char ** argv) {
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
