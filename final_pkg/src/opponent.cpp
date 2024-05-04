#include "opponent.hpp"

Opponent::Opponent() : Executer::Executer(){
    // refresh opponent's publisher and subscription
    pose_topic = "opp_racecar/odom";
    scan_topic = "opp_scan";
    drive_topic = "opp_drive";

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&Opponent::pose_callback, this, std::placeholders::_1)
    );
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&Opponent::scan_callback, this, std::placeholders::_1)
    );

    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10
    );

    // refresh opponent's frame id
    parent_frame_id = "map";
    child_frame_id = "opp_racecar/base_link";
}

Opponent::~Opponent(){

}

void Opponent::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    pure_pursuit_handler->update_params(
        this->get_parameter("pp_look_ahead_distance").as_double(),
        this->get_parameter("pp_kp").as_double(),
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y
    );

    pure_pursuit_handler->get_transform_stamp_W2L(parent_frame_id, child_frame_id, tf_buffer_pp_);

    pure_pursuit(pose_msg);
}

