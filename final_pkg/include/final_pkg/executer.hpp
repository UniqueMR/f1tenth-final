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
#include "rrt/rrt.hpp"

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

    // transformation
    geometry_msgs::msg::TransformStamped w2l_t, l2w_t;

    // topics
    std::string occupancy_grid_topic, pose_topic, scan_topic, state_topic, drive_topic, marker_topic, w2l_t_topic, l2w_t_topic;

    execState curr_state;

    // tf buffer and listener
    std::string parent_frame_id;
    std::string child_frame_id;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_pp_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_pp_{nullptr};
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_rrt_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_rrt_{nullptr};
    
    // publisher and subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::ConstSharedPtr laser_scan_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::ConstSharedPtr state_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::ConstSharedPtr w2l_t_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::ConstSharedPtr l2w_t_subscriber_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    //callbacks
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void state_callback(const std_msgs::msg::String::ConstSharedPtr state_msg); 
    void w2l_t_callback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr w2l_t_msg);
    void l2w_t_callback(const geometry_msgs::msg::TransformStamped::ConstSharedPtr l2w_t_msg);

    // state handler
    std::unique_ptr<purepursuitHandler> pure_pursuit_handler;
    std::unique_ptr<rrtHandler> rrt_handler;
    void pure_pursuit(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void rrt(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void blocking();

    // params
    double pp_look_ahead_distance;
    double pp_kp;
};