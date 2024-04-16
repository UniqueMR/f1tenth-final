#include "executer.hpp"

Executer::Executer()
: rclcpp::Node("Executer"){
    this->declare_parameter<bool>("online", false);
    online = this->get_parameter("online").as_bool();

    // initialize topic
    occupancy_grid_topic = "dynamic_map";
    pose_topic = (online) ? "pf/pose/odom" : "ego_racecar/odom";
    scan_topic = "/scan";

    // initialize publisher and subscriber
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&Executer::pose_callback, this, std::placeholders::_1)
    );
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&Executer::scan_callback, this, std::placeholders::_1)
    );


    return;
}

Executer::~Executer()
{
    return;
}

void Executer::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){

}

void Executer::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){

}

