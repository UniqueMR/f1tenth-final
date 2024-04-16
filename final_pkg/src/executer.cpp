#include "executer.hpp"

Executer::Executer()
: rclcpp::Node("Executer"){
    //declare parameters
    this->declare_parameter<bool>("online", false);
    online = this->get_parameter("online").as_bool();
    this->declare_parameter<double>("pp_look_ahead_distance", 2.0);

    // initialize topic
    occupancy_grid_topic = "dynamic_map";
    pose_topic = (online) ? "pf/pose/odom" : "ego_racecar/odom";
    scan_topic = "/scan";
    state_topic = "strategy";
    drive_topic = "drive";

    // initialize publisher and subscriber
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        pose_topic, 1, std::bind(&Executer::pose_callback, this, std::placeholders::_1)
    );
    laser_scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&Executer::scan_callback, this, std::placeholders::_1)
    );
    state_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        state_topic, 1, std::bind(&Executer::state_callback, this, std::placeholders::_1)
    );

    drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10
    );

    // initialize frames and tf buffer and listener
    parent_frame_id = "map";
    child_frame_id = (online) ? "laser" : "ego_racecar/base_link";
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //initialize the current state
    curr_state = execState::NORMAL;

    return;
}

Executer::~Executer()
{
    return;
}

void Executer::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    switch(curr_state){
        case execState::NORMAL:
            pure_pursuit();
            break;
        case execState::OVERTAKE:
            rrt();
            break;
        case execState::BLOCKING:
            blocking();
            break;
        case execState::INVALID:
            RCLCPP_WARN(this->get_logger(), "No strategy found!\n");
            break;   
    }
}

void Executer::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
}

void Executer::pure_pursuit(){
    RCLCPP_INFO(this->get_logger(), "select pure pursuit strategy...\n");
    // get params each stamp
    pp_look_ahead_distance = this->get_parameter("pp_look_ahead_distance").as_double();
}

void Executer::rrt(){
    RCLCPP_INFO(this->get_logger(), "select rrt strategy...\n");
}

void Executer::blocking(){
    RCLCPP_INFO(this->get_logger(), "select blocking strategy...\n");
}

void Executer::state_callback(const std_msgs::msg::String::ConstSharedPtr state_msg){
    curr_state = stringToState(state_msg);
    RCLCPP_INFO(this->get_logger(), "The current state changes to %s\n", state_msg->data.c_str());
}

execState stringToState(const std_msgs::msg::String::ConstSharedPtr str){
    static const std::map<std::string, execState> stateMap = {
        {"normal", execState::NORMAL},
        {"overtake", execState::OVERTAKE},
        {"blocking", execState::BLOCKING}
    };
    auto it = stateMap.find(str->data.c_str());
    return (it != stateMap.end()) ? it->second : execState::INVALID;
}