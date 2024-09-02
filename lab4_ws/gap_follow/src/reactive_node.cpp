#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <limits>  // For std::numeric_limits

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(x) ((x)/180.0*PI)

/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node 
{
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers

        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_marker", 10);
        

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    double distance_;
    double scan_min_angle_;
    double scan_max_angle_;
    double angle_increment_;
    double min_angle_;
    double max_angle_;
    double current_angle_;
    std::vector<double> ranges;

    /// ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void preprocess_lidar(float* ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        return;
    }

    void find_max_gap(std::vector<float> ranges, double indice)
    {   
        // Return the start index & end index of the max gap in free_space_ranges

        double gap_counter = 0;
        double gap_open_counter = 0;
        double gap_termination = 1234;
        

        RCLCPP_INFO(this->get_logger(), "find_max_gap:**");

        for (unsigned int i = 0; i < ranges.size(); i++)
        {
            if(ranges[i] = 1000000)
            {
                if(ranges[i+1] = 1000000)
                {
                    ranges[i] = gap_termination;
                    RCLCPP_INFO(this->get_logger(), "find_max_gap: FrontGap Termination Found!!!");
                }
                RCLCPP_INFO(this->get_logger(), "find_max_gap: Gap Found!!!");
                if(ranges[i+1] = 1000000)
                {
                    gap_open_counter++;
                    RCLCPP_INFO(this->get_logger(), "find_max_gap: Gap = '%f' Gaps", gap_open_counter);
                }
                else
                {
                    ranges[i] = gap_termination;
                    RCLCPP_INFO(this->get_logger(), "find_max_gap: BackGap Termination Found!!!");
                }

            }
            
        }
        return;
    }

    void find_best_point(double ranges, double indice)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively

        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        return;
    }

    void publish_marker(double angle) 
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = this->now();
        marker.ns = "lidar_scan";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = cos(angle);
        marker.pose.position.y = sin(angle);
        marker.pose.position.z = 1.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.lifetime = rclcpp::Duration(0.1);

        marker_pub_->publish(marker);
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
        auto range_data_tracker_ = scan_msg->ranges;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
	    double rbs = 150;
        double smallest_range_indx;
        double largest_range_indx;
        double largest_gap_;

        // Initialize the smallest and largest values
        float smallest_range = std::numeric_limits<float>::max();
        float largest_range = std::numeric_limits<float>::lowest();

        // Simplify lidar FOV

        min_angle_ = DEG2RAD(-70);
        max_angle_ = DEG2RAD(70);

        for (unsigned int i = 0; i < range_data_.size(); i++)
        {
            distance_ = scan_msg->ranges[i];
            angle_increment_ = scan_msg->angle_increment;
            scan_min_angle_ = scan_msg->angle_min;
            scan_max_angle_ = scan_msg->angle_max;
            current_angle_ = scan_min_angle_ + angle_increment_ * i;
            
            // Visual used for debug (Publish marker for the current angle)
            publish_marker(current_angle_);                              

            if (!std::isinf(distance_) && !std::isnan(distance_) && current_angle_ > min_angle_ && current_angle_ < max_angle_)
            {
                //RCLCPP_INFO(this->get_logger(), "lidar_callback: NO GAP! Range & Angle(deg)         = '%f' at '%f'", range_data_[i], RAD2DEG(current_angle_));
                //RCLCPP_INFO(this->get_logger(), "---------------------------ELIMINATING NO GAPS AND SETTING TO ZERO--------------------------------------------------");
    
                // Update smallest_range if the current_range is smaller
                if (range_data_[i] < smallest_range)
                {
                    smallest_range = range_data_[i];
                    smallest_range_indx = i;
                }

                // Update largest_range if the current_range is larger
                if (range_data_[i] > largest_range)
                {
                    largest_range = range_data_[i];
                    largest_range_indx = i;
                }
                if (i > 2)
                {
                  for (unsigned int i = 0; i < range_data_.size(); i++)
                  {
                    if(range_data_[i]> 2.0)
                    {
                        range_data_tracker_[i] = 1000000;
                    }
                  }
                }
            }
            range_data_tracker_[largest_range_indx] = largest_range;
            find_max_gap(range_data_tracker_,largest_range_indx);
        }

        // After the loop, you can use smallest_range and largest_range as needed
        RCLCPP_INFO(this->get_logger(), "lidar_callback: Smallest range value: '%f' meters at '%f degrees", smallest_range, scan_min_angle_ + angle_increment_ * smallest_range_indx);
        RCLCPP_INFO(this->get_logger(), "lidar_callback: Largest range value: '%f' meters at '%f degrees", largest_range, scan_min_angle_ + angle_increment_ * largest_range_indx);
        

        
        RCLCPP_INFO(this->get_logger(), "***********************************DISPLAYING FULL FINAL RANGE MEASUREMENTS BELOW********************************************");

        for (unsigned int i = 0; i < range_data_.size(); i++)
        {
            current_angle_ = scan_min_angle_ + angle_increment_ * i;
            if (current_angle_ > min_angle_ && current_angle_ < max_angle_)
            {
                RCLCPP_INFO(this->get_logger(), "* lidar_callback: Final range_data & Angle(deg; negative is to the right of front) = '%f'm at '%f' degrees: Gaps Tracker '%f'(GAPS at 1000) *", range_data_[i], RAD2DEG(scan_min_angle_ + angle_increment_ * i), range_data_tracker_[i]);
            }
        }
    
        drive_msg.drive.steering_angle = angle_increment_ * largest_range_indx;
        drive_msg.drive.speed = 1.0;
        ackermann_publisher_->publish(drive_msg);
    }

}; // End of class ReactiveFollowGap

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}