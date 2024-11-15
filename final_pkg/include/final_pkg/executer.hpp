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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For transforming ROS messages
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "utils/csv_loader.hpp"
#include "pure_pursuit/pure_pursuit.hpp"
#include "rrt/rrt.hpp"
#include "blocking/blocking.hpp"
#include <tf2/utils.h>

enum class execState {
    NORMAL,
    OVERTAKE,
    BLOCKING,
    BRAKING,
    INVALID
};

execState stringToState(const std_msgs::msg::String::ConstSharedPtr str);
class Executer : public rclcpp::Node {
public:
    Executer();
    virtual ~Executer();
protected:
    bool online;

    // topics
    std::string occupancy_grid_topic, pose_topic, scan_topic, state_topic, drive_topic, marker_topic, oppo_position_topic;

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
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::ConstSharedPtr oppo_position_subscriber_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    //callbacks
    virtual void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
    void state_callback(const std_msgs::msg::String::ConstSharedPtr state_msg); 
    void oppo_position_callback(const geometry_msgs::msg::PointStamped::ConstSharedPtr oppo_position_msg);

    // state handler
    std::unique_ptr<purepursuitHandler> pure_pursuit_handler;
    std::unique_ptr<rrtHandler> rrt_handler;
    std::unique_ptr<blockingHandler> blocking_handler;
    void pure_pursuit(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg, bool is_brake);
    void rrt(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    void blocking();

    void pack_transformation(geometry_msgs::msg::TransformStamped &t, std::string parent_frame_id, std::string child_frame_id, nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    // params
    double pp_look_ahead_distance;
    double pp_kp;
};

extern "C" void launch_hello_world_kernel();
extern "C" bool check_collision_cuda(double pta_x, double pta_y, double ptb_x, double ptb_y, 
                                 int check_pts_num, double origin_x, double origin_y, 
                                 double resolution, int width, int *map_data);
extern "C" void update_occupancy_grid_cuda(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> updated_map,
    geometry_msgs::msg::TransformStamped& transform,
    double look_ahead_dist, int bubble_offset
);