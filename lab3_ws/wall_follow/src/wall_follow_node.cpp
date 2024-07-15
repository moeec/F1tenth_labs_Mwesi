#include "rclcpp/rclcpp.hpp"
#include <string>
#include <chrono>
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


       /* Older code left just incase
       
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

        double angle_increment = angle/range_data.size();*/

       
        float range_measurement;
        float returned_range;
        double current_angle_; 

        for (unsigned int i = 0; i < size; i++)
        {
            range_measurement = range_data[i];
            current_angle_ = RAD2DEG(angle_increment_ * i);
            RCLCPP_INFO(this->get_logger(), "Current angle is = '%2f'", current_angle_);                           

            if (!std::isinf(range_measurement) && !std::isnan(range_measurement))
            {
                returned_range = range_data[i];

            }
        }

       
          
        return returned_range;
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

        angle_increment_ = scan_msg->angle_increment;
        angle_min_ = scan_msg->angle_min;
        auto range_data_ = scan_msg->ranges;
        size = range_data_.size();

        unsigned int b_index = (unsigned int)(floor((DEG2RAD(90) - angle_min_) / angle_increment_));
        double b_angle = DEG2RAD(90);        // 90.0 / 180.0 * PI; older method
        double a_angle = DEG2RAD(45);         // 45.0 / 180.0 * PI; older method
        unsigned int a_index;

        RCLCPP_INFO(this->get_logger(), "scan_callback:a_angle before if loop = '%2f'", a_angle);

        if (angle_min_ > DEG2RAD(45)) 
        {
            a_angle = angle_min_;
            a_index = 0;
        } 
        else 
        {
            a_index = (unsigned int)(floor((DEG2RAD(45) - angle_min_) / angle_increment_));
        }


        RCLCPP_INFO(this->get_logger(), "scan_callback:a_angle = '%2f'", a_angle);
        RCLCPP_INFO(this->get_logger(), "scan_callback:a_index = '%2f'", a_index);    

        float a_range = get_range(range_data_, size, a_angle);
        // double b_range = get_range();

        RCLCPP_INFO(this->get_logger(), "scan_callback:range returned = '%2f'", a_range);

        
        




        //auto range_array = scan_msg->ranges;
        //auto laser_scan = scan_msg;
        //auto ackermann_drive = ackermann_msgs::msg::AckermannDriveStamped();
        //auto angle_min_ = scan_msg->angle_min
        //auto angle_max_ = scan_msg->angle_max+

        double error = 0.0; // TODO: replace with error calculated by get_error()
        double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

    }

    rclcpp::TimerBase::SharedPtr ackermann_timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    double angle_increment_;
    double angle_min_;
    size_t size; 

};
int main(int argc, char ** argv) {
    
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}