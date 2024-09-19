#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define PI 3.1415927
#define RAD2DEG(x) ((x)*180. / PI)
#define DEG2RAD(x) ((x)*PI / 180.0)

class ReactiveFollowGap : public rclcpp::Node
{
public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("lidar_marker", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
    }

private:
    // ROS publishers and subscribers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    // Preprocess the LiDAR scan array
    std::vector<float> preprocess_lidar(const std::vector<float> &ranges, float range_min, float range_max)
    {
        std::vector<float> proc_ranges = ranges;

        // Apply smoothing filter (moving average with a window size of 3)
        for (size_t i = 1; i < ranges.size() - 1; ++i)
        {
            proc_ranges[i] = (ranges[i - 1] + ranges[i] + ranges[i + 1]) / 3.0;
        }

        // Reject high values
        for (size_t i = 0; i < proc_ranges.size(); ++i)
        {
            if (std::isnan(proc_ranges[i]) || proc_ranges[i] > range_max || proc_ranges[i] < range_min)
            {
                proc_ranges[i] = range_max;
            }
        }

        return proc_ranges;
    }

    // Find the closest point in the LiDAR scan
    int find_closest_point(const std::vector<float> &ranges)
    {
        return std::distance(ranges.begin(), std::min_element(ranges.begin(), ranges.end()));
    }

    // Set all points inside the safety bubble to zero
    void set_safety_bubble(std::vector<float> &ranges, int closest_point_idx, int bubble_size)
    {
        int start_idx = std::max(0, closest_point_idx - bubble_size);
        int end_idx = std::min(static_cast<int>(ranges.size()) - 1, closest_point_idx + bubble_size);

        for (int i = start_idx; i <= end_idx; ++i)
        {
            ranges[i] = 0.0;
        }
    }

    // Find the largest gap in the ranges array
    std::pair<int, int> find_max_gap(const std::vector<float> &ranges)
    {
        int max_start = 0;
        int max_end = 0;
        int max_length = 0;

        int current_start = -1;
        int current_length = 0;

        for (size_t i = 0; i < ranges.size(); ++i)
        {
            if (ranges[i] > 0.0)
            {
                if (current_start == -1)
                {
                    current_start = i;
                }
                current_length++;
            }
            else
            {
                if (current_length > max_length)
                {
                    max_length = current_length;
                    max_start = current_start;
                    max_end = i - 1;
                }
                current_start = -1;
                current_length = 0;
            }
        }

        // Check at the end of the array
        if (current_length > max_length)
        {
            max_length = current_length;
            max_start = current_start;
            max_end = ranges.size() - 1;
        }

        return std::make_pair(max_start, max_end);
    }

    // Find the best point in the largest gap
    int find_best_point(const std::vector<float> &ranges, int start_idx, int end_idx)
    {
        // Choose the furthest point within the gap
        auto max_elem_iter = std::max_element(ranges.begin() + start_idx, ranges.begin() + end_idx + 1);
        return std::distance(ranges.begin(), max_elem_iter);
    }

    // Publish a marker for visualization (optional)
    void publish_marker(float angle)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = this->now();
        marker.ns = "best_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Position at the unit circle for the angle
        marker.pose.position.x = cos(angle);
        marker.pose.position.y = sin(angle);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        marker.lifetime = rclcpp::Duration(0.1);

        marker_pub_->publish(marker);
    }

    // LiDAR callback function
    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        // Preprocess the LiDAR scan
        auto proc_ranges = preprocess_lidar(scan_msg->ranges, scan_msg->range_min, scan_msg->range_max);

        // Find the closest point
        int closest_point_idx = find_closest_point(proc_ranges);

        // Create a safety bubble around the closest point
        int bubble_size = static_cast<int>(std::ceil(DEG2RAD(10.0) / scan_msg->angle_increment)); // Bubble of 10 degrees
        set_safety_bubble(proc_ranges, closest_point_idx, bubble_size);

        // Find the largest gap
        auto[max_start_idx, max_end_idx] = find_max_gap(proc_ranges);

        // Find the best point in the largest gap
        int best_point_idx = find_best_point(proc_ranges, max_start_idx, max_end_idx);

        // Calculate the steering angle
        float steering_angle = scan_msg->angle_min + best_point_idx * scan_msg->angle_increment;

        // Publish the marker for visualization
        publish_marker(steering_angle);

        // Create and publish the drive message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->now();
        drive_msg.header.frame_id = "laser";

        // Set steering angle and speed
        drive_msg.drive.steering_angle = steering_angle;

        // Speed control based on steering angle
        float abs_steering_angle = std::abs(steering_angle);
        if (abs_steering_angle > DEG2RAD(20.0))
        {
            drive_msg.drive.speed = 0.5;
        }
        else if (abs_steering_angle > DEG2RAD(10.0))
        {
            drive_msg.drive.speed = 1.0;
        }
        else
        {
            drive_msg.drive.speed = 1.5;
        }

        ackermann_publisher_->publish(drive_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}

