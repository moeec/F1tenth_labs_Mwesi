#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

// ---- Constants / helpers (C++14 friendly) -----------------------------------
constexpr double PI = 3.14159265358979323846;

inline double rad2deg(double x) { return x * 180.0 / PI; }
inline double deg2rad(double x) { return x / 180.0 * PI; }

const float MAX_STEER = deg2rad(20.0f); // tune to your car
// ---- Node -------------------------------------------------------------------
class ReactiveFollowGap : public rclcpp::Node
{
public:
  ReactiveFollowGap() : rclcpp::Node("reactive_node")
  {
    // Publishers (explicit QoS; tune as needed)
    auto drive_qos  = rclcpp::QoS(100).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    auto marker_qos = rclcpp::QoS(3).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                   .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, drive_qos);
    marker_pub_          = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic_, marker_qos);

    // Subscription using a lambda (no std::bind / placeholders)
    auto scan_qos = rclcpp::QoS(100).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
                                .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic_, scan_qos,
        [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          this->lidar_callback(msg);
        });

    RCLCPP_INFO(this->get_logger(), "ReactiveFollowGap node started (C++14).");
  }

private:
  // Topics
  std::string lidarscan_topic_ = "/scan";
  std::string drive_topic_     = "/drive";
  std::string marker_topic_    = "lidar_marker";

  // ROS interfaces
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr              scan_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr             marker_pub_;

  // --- Helpers ---------------------------------------------------------------

  // Simple moving-average smoothing + range clipping
  std::vector<float> preprocess_lidar(const std::vector<float> &ranges,float range_min, float range_max)
  {
  //RCLCPP_INFO(this->get_logger(),"Inside Preprocessing function");
    std::vector<float> proc = ranges;

    if (ranges.size() >= 3) 
    {
      for (size_t i = 1; i + 1 < ranges.size(); ++i) {
        proc[i] = (ranges[i - 1] + ranges[i] + ranges[i + 1]) / 3.0f;
        }
    }

    // Handle edges (optional): copy neighbors if available
    if (!ranges.empty()) {
      proc[0] = (ranges.size() > 1) ? (ranges[0] + ranges[1]) / 2.0f : ranges[0];
      proc.back() = (ranges.size() > 1) ? (ranges[ranges.size()-2] + ranges.back()) / 2.0f : ranges.back();
    }

    // Reject invalid/too-large/too-small values -> treat as open space (range_max)
    for (size_t i = 0; i < proc.size(); ++i) {
      float &v = proc[i];
      if (std::isnan(v) || v > range_max || v < range_min) {
        v = range_max;
      }
    }
    return proc;
  }

  // Zero out (mark blocked) a region around the closest obstacle
  void set_safety_bubble(std::vector<float> &ranges, int closest_idx, int bubble_size)
  {
    const int n = static_cast<int>(ranges.size());
    const int start = std::max(0, closest_idx - bubble_size);
    const int end   = std::min(n - 1, closest_idx + bubble_size);
    for (int i = start; i <= end; ++i) {
      ranges[i] = 0.0f;  // 0.0 == blocked in this convention
    }
  }

  // Find the longest contiguous run of > 0.0 values (the largest gap)
  std::pair<int, int> find_max_gap(const std::vector<float> &ranges)
  {
    int max_s = 0, max_e = -1, max_len = 0;
    int cur_s = -1, cur_len = 0;

    for (int i = 0; i < static_cast<int>(ranges.size()); ++i) {
      if (ranges[i] > 0.0f) {
        if (cur_s == -1) cur_s = i;
        ++cur_len;
      } else {
        if (cur_len > max_len) {
          max_len = cur_len;
          max_s = cur_s;
          max_e = i - 1;
        }
        cur_s = -1;
        cur_len = 0;
      }
    }
    // tail check
    if (cur_len > max_len) {
      max_len = cur_len;
      max_s = cur_s;

      max_e = static_cast<int>(ranges.size()) - 1;
    }
    if (max_len == 0) return std::pair<int,int>(0, -1); // no free gap
    return std::pair<int,int>(max_s, max_e);
  }

  // Choose the "best" point in [start_idx, end_idx]; here: farthest range
  int find_best_point(const std::vector<float> &ranges, int start_idx, int end_idx)
  {
    const int n = static_cast<int>(ranges.size());
    if (n == 0) return 0;

    // Clamp indices manually (C++14: no std::clamp)
    if (start_idx < 0) start_idx = 0;
    if (end_idx   < 0) end_idx   = 0;
    if (start_idx >= n) start_idx = n - 1;
    if (end_idx   >= n) end_idx   = n - 1;

    if (start_idx > end_idx) {
      return start_idx;  // degenerate range; best we can do
    }

    std::vector<float>::const_iterator it =
        std::max_element(ranges.begin() + start_idx, ranges.begin() + end_idx + 1);
    return static_cast<int>(std::distance(ranges.begin(), it));
  }

  // Publish a sphere marker at unit circle location for the chosen angle
  void publish_marker(double angle)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser";                 // keep consistent with drive frame
    marker.header.stamp    = this->now();
    marker.ns              = "lidar_scan";
    marker.id              = 0;
    marker.type            = visualization_msgs::msg::Marker::SPHERE;
    marker.action          = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = std::cos(angle);
    marker.pose.position.y = std::sin(angle);
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

    // Lifetime: 0.1s (builtin_interfaces/Duration fields)
    marker.lifetime.sec     = 0;
    marker.lifetime.nanosec = 100000000;

    marker_pub_->publish(marker);
  }

  // --- Subscription callback -------------------------------------------------
  void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
  {
    // 1) Preprocess
    std::vector<float> proc_ranges = preprocess_lidar(scan_msg->ranges, scan_msg->range_min, scan_msg->range_max);
    
    //for debugging
    //for (int i = 0; i < static_cast<int>(proc_ranges.size()); ++i) {
      //RCLCPP_INFO(this->get_logger(), "1) Preprocess %.6f %6f.", proc_ranges[i],rad2deg(scan_msg->angle_min + (i*scan_msg->angle_increment)));
    //}
 
  
    

    // 2) Closest obstacle
    /* int closest_idx = static_cast<int>(
        std::distance(proc_ranges.begin(),
                      std::min_element(proc_ranges.begin(), proc_ranges.end()))); */
                      
    std::size_t closest_idx = 0;
    float       closest_val = std::numeric_limits<float>::infinity();

    if (!proc_ranges.empty()) {
        RCLCPP_INFO(this->get_logger(), "2) Inside Closest Obstacle assessor----------------------------------------------------------------- .");
        
        //for (int i = 0; i < static_cast<int>(proc_ranges.size()); ++i) {
        //RCLCPP_INFO(this->get_logger(), "2) Closest Obstacle %.6f %6f.", proc_ranges[i],rad2deg(scan_msg->angle_min + (i*scan_msg->angle_increment)));
        //}
    	auto it = std::min_element(proc_ranges.begin(), proc_ranges.end());  
        //RCLCPP_INFO(this->get_logger(), "2) Closest Obstacle proc_ranges(): %.6f meters away.", *it);
        //RCLCPP_INFO(this->get_logger(), "2) Closest Obstacle *it: %.6f meters away.", *it);
        
        closest_idx = static_cast<std::size_t>(it - proc_ranges.begin());
    	closest_val = *it;
        //RCLCPP_INFO(this->get_logger(), "2) Closest value %6f.", closest_val);
    	}

    //RCLCPP_INFO(this->get_logger(), "2) Closest Index %zu.", closest_idx);
    RCLCPP_INFO(this->get_logger(), "2) Closest at  %.6f.degrees", rad2deg(scan_msg->angle_min + (closest_idx*scan_msg->angle_increment)));   
    RCLCPP_INFO(this->get_logger(), "2) Closest Value %.6f meters.", closest_val);
    

    // 3) Safety bubble (10 degrees around closest point)

    int bubble_size = static_cast<int>(std::ceil(deg2rad(10.0) / scan_msg->angle_increment));
    set_safety_bubble(proc_ranges, closest_idx, bubble_size);
    
    //RCLCPP_INFO(this->get_logger(), "2) Bubble Size %.5f.", bubble_size);
    //for debugging
    //for (int i = 0; i < static_cast<int>(proc_ranges.size()); ++i) {  
    //    RCLCPP_INFO(this->get_logger(), "1) Processed range With Bubble at %.6f degrees, %.4f meters.", rad2deg(i*scan_msg->angle_increment), proc_ranges[i]);
    //    }
    

    // 4) Largest gap
    std::pair<int,int> gap = find_max_gap(proc_ranges);
    int gap_s = gap.first;
    int gap_e = gap.second;

    if (gap_e < gap_s) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "No free gap detected; sending stop.");
      publish_stop();
      return;
    }

    // 5) Best point (farthest) within gap
    int best_idx = find_best_point(proc_ranges, gap_s, gap_e);
    
    RCLCPP_INFO(this->get_logger(), "5) Best point (farthest) within gap %.6f.", best_idx);
    RCLCPP_INFO(this->get_logger(), "5) best_idx range at %.6f degrees, %.4f meters.", rad2deg(scan_msg->angle_min+(best_idx*scan_msg->angle_increment)), proc_ranges[best_idx]);

    // 6) Index -> angle (radians in laser frame)
    float steering_angle = scan_msg->angle_min + best_idx * scan_msg->angle_increment;
    
     

    // 7) Visualize in Rviz, for simultaion
    // publish_marker(steering_angle);

    // 8) Command
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp    = this->now();
    drive_msg.header.frame_id = "laser";

    drive_msg.drive.steering_angle = steering_angle;

    RCLCPP_INFO(this->get_logger(), "6) Steering Angle %.6f.", rad2deg(steering_angle));

    // Speed scheduling by steering demand
    float abs_angle = std::fabs(steering_angle);
    if (abs_angle > deg2rad(20.0)) {
      //drive_msg.drive.speed = 0.10f;
      RCLCPP_INFO(this->get_logger(), "6.5) STEERING Angle inside steer %.6f.\033[0m", rad2deg(steering_angle));
    } else if (abs_angle > deg2rad(10.0)) {
      //drive_msg.drive.speed = 0.35f;
    } else {
     // drive_msg.drive.speed = 0.52f;
    }

    drive_msg.drive.steering_angle = steering_angle;
    
   // manual clamp
   //if (steering_angle >  MAX_STEER) steering_angle =  MAX_STEER;
   //if (steering_angle < -MAX_STEER) steering_angle = -MAX_STEER;
    
    ackermann_publisher_->publish(drive_msg);
  }

  void publish_stop()
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp    = this->now();
    msg.header.frame_id = "laser";
    //msg.drive.steering_angle = 0.0f;
    msg.drive.speed          = 0.0f;
    ackermann_publisher_->publish(msg);
  }
};

// ---- Main ------------------------------------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReactiveFollowGap>());
  rclcpp::shutdown();
  return 0;
}
