#pragma once

#include <memory> // for std::unique_ptr
#include <string>
#include <string>
#include "utils/csv_loader.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"

class purepursuitHandler{
public:
    purepursuitHandler(std::string waypoints_path);
    virtual ~purepursuitHandler();
    void update_params(double look_ahead_dist, double kp, double x, double y);
    void get_transform_stamp(std::string parent_frame_id, std::string child_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer_);
private:
    double look_ahead_dist;
    double kp;
    double speed;

    std::unique_ptr<wayPointLoader> dataloader;
    std::vector<wayPoint> way_points;

    geometry_msgs::msg::PointStamped curr_pt_world, next_pt_world, curr_pt_local, next_pt_local;
    geometry_msgs::msg::TransformStamped t;
};