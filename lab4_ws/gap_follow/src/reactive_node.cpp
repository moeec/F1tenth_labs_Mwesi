#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <limits>  // For std::numeric_limits

// Initialize the smallest value to the largest possible float value
float smallest_range = std::numeric_limits<float>::max();

for (unsigned int i = 0; i < range_data_.size(); i++)
{
    float current_range = range_data_[i];
    
    // Update smallest_range if the current_range is smaller
    if (current_range < smallest_range)
    {
        smallest_range = current_range;
    }
    
    RCLCPP_INFO(this->get_logger(), "| lidar_callback: Final range_data & Angle(deg) = '%f' m at '%f' degrees |", current_range, RAD2DEG(scan_min_angle_ + angle_increment_ * i));
    
    // Publish marker for the current angle used for debugging
    publish_marker(scan_min_angle_ + angle_increment_ * i);
}

// After the loop, you can use smallest_range as needed
RCLCPP_INFO(this->get_logger(), "Smallest range value: '%f' meters", smallest_range);


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
	    double rbs = 150;
	    //ranges = std::vector<double>(std::begin(scan_msg.ranges), std::end(scan_msg.ranges));

        // Simplify lidar FOV

        min_angle_ = DEG2RAD(-70);
        max_angle_ = DEG2RAD(70);

        //RCLCPP_INFO(this->get_logger(), "lidar_callback: min_angle = DEG2RAD(-70) = '%2f'", min_angle_); //for debugging
        //RCLCPP_INFO(this->get_logger(), "lidar_callback: scan min_angle = DEG2RAD(-70) = '%2f'", scan_msg->angle_min); //for debugging
        //RCLCPP_INFO(this->get_logger(), "lidar_callback: max_angle = DEG2RAD(-70) = '%2f'", max_angle_); //for debugging
        //RCLCPP_INFO(this->get_logger(), "lidar_callback: scan_msg->angle_increment = '%2f'", scan_msg->angle_increment); //for debugging
        

        
        //unsigned int min_index = (unsigned int)(std::floor((min_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));
        //unsigned int max_index = (unsigned int)(std::ceil((max_angle_ - scan_msg->angle_min) / scan_msg->angle_increment));

        //RCLCPP_INFO(this->get_logger(), "lidar_callback: min_index = '%2f'", min_index); //for debugging
        //RCLCPP_INFO(this->get_logger(), "lidar_callback: max_index = '%2f'", max_index); //for debugging


        //  Find closest & furthest detected point (LiDAR)
        double min_range = 0.025;
        //double max_range = scan_msg->range_max;

	    // Initialize the smallest and largest values
        float smallest_range = std::numeric_limits<float>::max();
        float largest_range = std::numeric_limits<float>::lowest();



        for (unsigned int i = 0; i < range_data_.size(); i++)
        {
            distance_ = scan_msg->ranges[i];
            angle_increment_ = scan_msg->angle_increment;
            scan_min_angle_ = scan_msg->angle_min;
            scan_max_angle_ = scan_msg->angle_max;
            current_angle_ = scan_min_angle_ + angle_increment_ * i;
            //range_rate_ = cos(current_angle_) * v_x + sin(current_angle_) * v_y;                               

            if (!std::isinf(distance_) && !std::isnan(distance_) && current_angle_ > min_angle_ && current_angle_ < max_angle_)
            {
                if (range_data_[i] < smallest_range)   
                {
                    RCLCPP_INFO(this->get_logger(), "lidar_callback: NO GAP! i value is now             = '%f'", i);
                    RCLCPP_INFO(this->get_logger(), "lidar_callback: NO GAP! Range & Angle(deg)         = '%f' at '%f'", range_data_[i], RAD2DEG(current_angle_));
                    RCLCPP_INFO(this->get_logger(), "---------------------------ELIMINATING NO GAPS AND SETTING TO ZERO--------------------------------------------------");
                    
                    float current_range = range_data_[i];
    
                    // Update smallest_range if the current_range is smaller
                    if (current_range < smallest_range)
                    {
                        smallest_range = current_range;
                    }

                    // Update largest_range if the current_range is larger
                    if (current_range > largest_range)
                    {
                        largest_range = current_range;
                    }
                    range_data_[i] = 0;
                    RCLCPP_INFO(this->get_logger(), "lidar_callback: NO GAP! New Range range            = '%f'", range_data_[i]);
                    //drive_msg.drive.speed = 2.0;
                    //ackermann_publisher_->publish(drive_msg);
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "lidar_callback: Possible GAP! Range: = '%f' at '%f'", range_data_[i], RAD2DEG(current_angle_));
			
			        // After the loop, you can use smallest_range and largest_range as needed
                    RCLCPP_INFO(this->get_logger(), "Smallest range value: '%f' meters", smallest_range);
                    RCLCPP_INFO(this->get_logger(), "Largest range value: '%f' meters", largest_range);

                }
            }   
        }

        RCLCPP_INFO(this->get_logger(), "***********************************DISPLAYING FULL FINAL RANGE MEASUREMENTS BELOW********************************************");

	    // Initialize the smallest value to the largest possible float value
        float smallest_range = std::numeric_limits<float>::max();

        for (unsigned int i = 0; i < range_data_.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "| lidar_callback: Final range_data & Angle(deg)                = '%f'm at '%f' degrees |", range_data_[i], RAD2DEG(scan_min_angle_ + angle_increment_ * i));
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
        marker.pose.position.z = 0.0;
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
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}
