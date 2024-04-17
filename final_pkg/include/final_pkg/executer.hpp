// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
#pragma once

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <fstream>
#include <stack>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming ROS messages
#include <tf2/LinearMath/Quaternion.h>

#include "utils/csv_loader.hpp"
#include "pure_pursuit/pure_pursuit.hpp"

/// CHECK: include needed ROS msg type headers and libraries

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y; // not sure
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

enum class execState {
    NORMAL,
    OVERTAKE,
    BLOCKING,
    INVALID
};

execState stringToState(const std_msgs::msg::String::ConstSharedPtr str);
class Executer : public rclcpp::Node {
public:
    Executer();
    virtual ~Executer();
private:
    bool online;

    // topics
    std::string occupancy_grid_topic, pose_topic, scan_topic, state_topic, drive_topic;

    execState curr_state;

    // tf buffer and listener
    std::string parent_frame_id;
    std::string child_frame_id;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    
    // publisher and subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr state_subscriber_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::ConstSharedPtr drive_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::ConstSharedPtr marker_publisher_;

    //callbacks
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void state_callback(const std_msgs::msg::String::ConstSharedPtr state_msg); 

    // state handler
    std::unique_ptr<purePursuitHandler> pure_pursuit_handler;
    void pure_pursuit();
    void rrt();
    void blocking();

    // params
    double pp_look_ahead_distance;
    double pp_kp;
};