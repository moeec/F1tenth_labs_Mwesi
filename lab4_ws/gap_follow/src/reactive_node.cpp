#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(x) ((x)/180.0*PI)

/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));

    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    double distance_;
    double angle_;
    double angle_increment_;
    double min_angle_;
    double max_angle_;
    double current_angle_;
    std::vector<double> ranges;

    /// ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        return;
    }

    void find_max_gap(float* ranges, int* indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        return;
    }

    void find_best_point(float* ranges, int* indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR

        // Eliminate all points inside 'bubble' (set them to zero) 

        // Find max length gap 

        // Find the best point in the gap 

        // Publish Drive message

        auto range_data_ = scan_msg->ranges;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
	    //ranges = std::vector<double>(std::begin(scan_msg.ranges), std::end(scan_msg.ranges));

        // Simplify lidar FOV

        min_angle_ = DEG2RAD(-70);
        max_angle_ = DEG2RAD(70);
        unsigned int min_index = (unsigned int)(std::floor((min_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));
        unsigned int max_index = (unsigned int)(std::ceil((max_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));

        RCLCPP_INFO(this->get_logger(), "min_index = '%2f'",min_index);     //for debugging

        for (unsigned int i = min_index; i <= max_index; i++) 
        {
            if (std::isinf(range_data_[i]) || std::isnan(range_data_[i])) 
            {
                ranges[i] = 0.0;
            } 
            else if (range_data_[i] > scan_msg->range_max) 
            {
                ranges[i] = scan_msg->range_max;
            }
        }



     /* older method used in previous code.


        for (unsigned int i = 0; i < range_data.size(); i++)
        {
            distance_ = scan_msg->ranges[i];
            angle_increment_ = scan_msg->angle_increment;
            angle_ = scan_msg->angle_min;
            current_angle_ = angle_ + angle_increment_ * i;

            if (!std::isinf(distance_) && !std::isnan(distance_))
            {
		ranges[i] = scan_msg.range_max;
             
                RCLCPP_WARN(this->get_logger(), "Scan Distance is inf or NaN");
            }

	    
        }
    */
    }

};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}