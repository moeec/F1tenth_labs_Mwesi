// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()) {

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need
    ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 1000);
    waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("waypoint_marker", 1000);
    static_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("static_marker", 1000);
    dynamic_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("dynamic_marker", 1000);
    points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("points_marker", 1000);

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // TODO: create a occupancy grid

    /* 500 outer elements  (y-index) X 200 inner elements (x-index)
      n = 500, so the outer vector size() and capacity() are both set to 500.
      A temporary inner vector is built: 200 bool entries initialised to true.
      C++ then copies that same inner vector 500 times (via std::vector copy-constructor) into the outer container.
      occupancy_grids_prior.size() = 500
      occupancy_grids_prior[i].size() 200 for any valid i
      All 100 000 entries (500 x 200) start out as true.
    */

    previous_occupancy_grid = std::vector<std::vector<bool>> (500,
    std::vector<bool>(200, true));


    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // TODO: update your occupancy grid

    RCLCPP_INFO(this->get_logger(), "scan");
    RCLCPP_INFO(this->get_logger(), "heading: %.2f",heading_current);

    auto range_data_ = scan_msg->ranges;
    auto angle_min_ = scan_msg->angle_min;
    auto angle_increment_ = scan_msg->angle_increment;
    double lidar_offset = 0.3;

    current_occupancy_grid= previous_occupancy_grid;

    double lidar_pcx = (x_current + lidar_offset) * std::cos(heading_current);
    double lidar_pcy = (y_current + lidar_offset) * std::sin(heading_current);

    for (unsigned int i = 0; i < range_data_.size(); i++) 
    {
        if (!std::isinf(range_data_[i]) && !std::isnan(range_data_[i])) 
        {
            double distance = range_data_[i];
            double car_angle = angle_min_ + angle_increment_ * i;
            double map_angle = car_angle + heading_current;
            double obstacle_detect_x = lidar_pcx + distance * std::cos(map_angle);
            double obstacle_detect_y = lidar_pcy + distance * std::sin(map_angle);

            for (unsigned int j = 0; i < current_occupancy_grid.size(); i++)         // 500 rows
            {
                for (unsigned int j = 0; i < current_occupancy_grid[0].size(); i++)  // 200 columns
                {
                    current_occupancy_grid[obstacle_detect_x ][obstacle_detect_y] = false;
                }

            }

        }
    }

    for (size_t i = 0; i < current_occupancy_grid.size(); ++i) 
    {
        std::string row_str;

        for (size_t j = 0; j < current_occupancy_grid[i].size(); ++j) 
        {
            row_str += current_occupancy_grid[i][j] ? '#' : '.';  // '#' for occupied, '.' for free
        }
        RCLCPP_INFO(rclcpp::get_logger("occupancy_grid"), "%s", row_str.c_str());
    }

}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //
    //RCLCPP_INFO(rclcpp::get_logger("RRT"), "nav_msgs");

    // tree as std::vector
    std::vector<RRT_Node> tree;

    auto position_odom = pose_msg->pose.pose.position;
    auto orientation_odom = pose_msg->pose.pose.orientation;

    double siny_cosp = 2.0 * (orientation_odom.w * orientation_odom.z + orientation_odom.x * orientation_odom.y);
    double cosy_cosp = 1.0 - 2.0 * (orientation_odom.y * orientation_odom.y + orientation_odom.z * orientation_odom.z);
    heading_current = std::atan2(siny_cosp, cosy_cosp);

    // TODO: fill in the RRT main loop

    /*for(size_t i = 0; i < &sampled_point.size(); i++)
    {

    }*/

    // path found as Path message

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = rclcpp::Clock().now();
 
    marker.ns = "dynamic_marker";
    marker.id = 0;

    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = 1;
    marker.pose.position.y = 1;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;   // Don't forget to set the alpha!

    marker.lifetime = rclcpp::Duration(0,0);

    dynamic_pub_->publish(marker);

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    
    // need to create area around me
    x_dist = std::uniform_real_distribution<>(x_limit_bottom, x_limit_top);
    y_dist = std::uniform_real_distribution<>(y_limit_right, y_limit_left);
    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree
    /*
    Explored points V:
    O1 (0, 0)      O2 (5, 0)      O3 (3, 4)

    New sample x = (4, 1)

    Compute distances:
    ‣ ||x - O1|| = sqrt((4-0)² + (1-0)²) = sqrt(17)
    ‣ ||x - O2|| = sqrt((4-5)² + (1-0)²) = sqrt(2)
    ‣ ||x - O3|| = sqrt((4-3)² + (1-4)²) = sqrt(10)

    argmin = O2 (closest)

    So:
    Nearest(G, x) = O2
    */
    int nearest_node = 0;
    // TODO: fill in this method
    for(size_t i = 0; i < tree.size(); i++)
    {
        //compute distances

        double x = sqrt(std::pow((sampled_point[0]-tree[i].x),2)+std::pow((sampled_point[1]-tree[i].y),2));

    // Code to be executed
    }
    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    RRT_Node new_node;
    // TODO: fill in this method

    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;

    // Rotate waypoint position into vehicle frame
    double local_x =  dx * cos(-heading_current) - dy * sin(-heading_current);
    double local_y =  dx * sin(-heading_current) + dy * cos(-heading_current);
    double euclidean_distance = dx*cos(dy);

    // Steering angle calculation is always atan2(local_y, local_x)
    steering_angle = atan2(local_y, local_x);

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    bool collision = false;
    // TODO: fill in this method

    return collision;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    // TODO: fill in this method

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}
