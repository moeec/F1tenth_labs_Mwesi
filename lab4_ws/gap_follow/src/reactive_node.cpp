#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
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
            "/scan", 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    double distance_;
    double angle_;
    double angle_increment_;
    double current_angle_;
    std::vector<double> ranges;

    /// TODO: create ROS subscribers and publishers

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
	ranges = std::vector<double>(std::begin(scan_msg.ranges), std::end(scan_msg.ranges));    

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



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
