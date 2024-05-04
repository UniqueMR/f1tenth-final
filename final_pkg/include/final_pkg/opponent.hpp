#pragma once

#include "executer.hpp"
#include "rclcpp/rclcpp.hpp"

class Opponent : public Executer{
public:
    Opponent();
    virtual ~Opponent();    

protected:
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) override;
};