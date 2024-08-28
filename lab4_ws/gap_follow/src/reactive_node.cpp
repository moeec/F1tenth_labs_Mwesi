#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180./PI)
#define DEG2RAD(x) ((x)/180.0*PI)

class ReactiveFollowGap : public rclcpp::Node {
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_marker", 10);
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

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        auto range_data_ = scan_msg->ranges;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        min_angle_ = DEG2RAD(-70);
        max_angle_ = DEG2RAD(70);

        double min_range = 0.025;

        for (unsigned int i = 0; i < range_data_.size(); i++)
        {
            distance_ = scan_msg->ranges[i];
            angle_increment_ = scan_msg->angle_increment;
            scan_min_angle_ = scan_msg->angle_min;
            scan_max_angle_ = scan_msg->angle_max;
            current_angle_ = scan_min_angle_ + angle_increment_ * i;

            if (!std::isinf(distance_) && !std::isnan(distance_) && current_angle_ > min_angle_ && current_angle_ < max_angle_)
            {
                if (range_data_[i] < min_range)   
                {
                    range_data_[i] = 0;
                }
            }   
        }

        // Publish marker for the current angle
        publish_marker(current_angle_);
    }

    void publish_marker(double angle) 
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser_frame";
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
