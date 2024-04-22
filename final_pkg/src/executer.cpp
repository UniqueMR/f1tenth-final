#include "executer.hpp"

Executer::Executer()
: rclcpp::Node("Executer"){
    // declare parameters
    this->declare_parameter<bool>("online", false);
    online = this->get_parameter("online").as_bool();
    this->declare_parameter<std::string>("waypoints_path", "");

    // pure pursuit parameters
    this->declare_parameter<double>("pp_look_ahead_distance", 2.0);
    this->declare_parameter<double>("pp_high_speed", 4.5);
    this->declare_parameter<double>("pp_medium_speed", 1.5);
    this->declare_parameter<double>("pp_low_speed", 1.0);
    this->declare_parameter<double>("pp_kp", 0.5);

    // rrt parameters
    this->declare_parameter<double>("rrt_look_ahead_dist", 6.0);
    this->declare_parameter<double>("rrt_track_dist", 1.5);
    this->declare_parameter<int>("rrt_iter", 25);
    this->declare_parameter<int>("rrt_check_pts_num", 200);
    this->declare_parameter<double>("rrt_max_expansion_dist", 0.75);
    this->declare_parameter<int>("rrt_obs_clear_rate", 10);
    this->declare_parameter<double>("rrt_kp", 1.0);

    this->declare_parameter<double>("rrt_high_speed", 3.0);
    this->declare_parameter<double>("rrt_medium_speed", 3.0);
    this->declare_parameter<double>("rrt_low_speed", 3.0);

    this->declare_parameter<int>("rrt_bubble_offset", 1);
    this->declare_parameter<bool>("rrt_star_enable", true);
    this->declare_parameter<double>("rrt_search_radius", 0.5);
    this->declare_parameter<double>("rrt_hit_dist", 0.05);
    this->declare_parameter<bool>("rrt_ema_enable", true);
    this->declare_parameter<double>("rrt_ema_alpha", 0.2);

    // initialize topic
    occupancy_grid_topic = "dynamic_map";
    pose_topic = (online) ? "pf/pose/odom" : "ego_racecar/odom";
    scan_topic = "/scan";
    state_topic = "strategy";
    drive_topic = "drive";
    marker_topic = "visualization_marker";

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
    occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        occupancy_grid_topic, 10
    );
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);


    // initialize frames and tf buffer and listener
    parent_frame_id = "map";
    child_frame_id = (online) ? "laser" : "ego_racecar/base_link";
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // initialize state handler
    pure_pursuit_handler = std::make_unique<purepursuitHandler>(
        this->get_parameter("waypoints_path").as_string().c_str()
    );

    rrt_handler = std::make_unique<rrtHandler>(
        this->get_parameter("waypoints_path").as_string().c_str(),
        parent_frame_id.c_str()
    );   

    //initialize the current state
    curr_state = execState::NORMAL;

    return;
}

Executer::~Executer()
{
    return;
}

void Executer::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    pure_pursuit_handler->update_params(
        this->get_parameter("pp_look_ahead_distance").as_double(),
        this->get_parameter("pp_kp").as_double(),
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y
    );

    pure_pursuit_handler->get_transform_stamp_W2L(parent_frame_id, child_frame_id, tf_buffer_);
    rrt_handler->get_transform_stamp_L2W(parent_frame_id, child_frame_id, tf_buffer_);

    switch(curr_state){
        case execState::NORMAL:
            pure_pursuit(pose_msg);
            break;
        case execState::OVERTAKE:
            rrt(pose_msg);
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
    rrt_handler->update_occupancy_grid(
        scan_msg,
        this->get_parameter("rrt_look_ahead_dist").as_double(),
        this->get_parameter("rrt_bubble_offset").as_int(),
        this->get_parameter("rrt_obs_clear_rate").as_int()
    );
    rrt_handler->updated_map->header.stamp = this->now();
    occupancy_grid_publisher_->publish(*(rrt_handler->updated_map));
}

void Executer::pure_pursuit(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    RCLCPP_INFO(this->get_logger(), "select pure pursuit strategy...\n");
    // compute the next look ahead point
    pure_pursuit_handler->get_lookahead_pt();

    double steeringAngle = pure_pursuit_handler->gen_steer_ang();

    // TODO: publish drive message, don't forget to limit the steering angle.
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    if (steeringAngle < 0.0)    drive_msg.drive.steering_angle = std::max(steeringAngle, -0.349);
    else    drive_msg.drive.steering_angle = std::min(steeringAngle, 0.349);

    drive_msg.drive.steering_angle = (steeringAngle < -0.349) ? -0.349 : ((steeringAngle > 0.349) ? 0.349 : steeringAngle);

    drive_msg.drive.speed = (drive_msg.drive.steering_angle < 0.1 && drive_msg.drive.steering_angle > -0.1) ? this->get_parameter("pp_high_speed").as_double() : ((drive_msg.drive.steering_angle > 0.2 || drive_msg.drive.steering_angle < -0.2) ? this->get_parameter("pp_low_speed").as_double() : this->get_parameter("pp_medium_speed").as_double());

    drive_publisher_->publish(drive_msg);
}

void Executer::rrt(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    RCLCPP_INFO(this->get_logger(), "select rrt strategy...\n");
    
    if(rrt_handler->clear_state)    return;

    rrt_handler->init_tree(pose_msg);
    
    std::vector<double> target_pt_world = rrt_handler->get_target_pt(
        pose_msg, pure_pursuit_handler->t, this->get_parameter("rrt_look_ahead_dist").as_double());
    rrt_handler->visualize_target(target_pt_world);
    marker_publisher_->publish(rrt_handler->visualized_target);

    std::vector<RRT_Node> path; 
    RRT_Node node_tracker;

    int cnt;
    for(cnt = 0; cnt < this->get_parameter("rrt_iter").as_int(); cnt++){
        std::vector<double> sampled_node_pt = rrt_handler->sample(
            this->get_parameter("rrt_look_ahead_dist").as_double()
        );

        if(rrt_handler->is_collide(sampled_node_pt))    continue;

        int nearest_node_id = rrt_handler->nearest(sampled_node_pt);

        RRT_Node new_node = rrt_handler->steer(
            nearest_node_id, sampled_node_pt,
            this->get_parameter("rrt_max_expansion_dist").as_double()
        );

        if(this->get_parameter("rrt_star_enable").as_bool()){
            new_node.cost = rrt_handler->cost(new_node);
            std::vector<int> neighbor_indices = rrt_handler->near(new_node, this->get_parameter("rrt_search_radius").as_double());
            std::vector<bool> neighbor_collided;
            int best_neighbor_idx = rrt_handler->link_best_neighbor(new_node, neighbor_indices, neighbor_collided, this->get_parameter("rrt_check_pts_num").as_int());
            if(rrt_handler->check_collision(new_node.parent, new_node, this->get_parameter("rrt_check_pts_num").as_int()))
                continue;
            if(best_neighbor_idx != -1)
                rrt_handler->rearrange_tree(best_neighbor_idx, neighbor_indices, neighbor_collided, new_node);
        }
        else{
            if(rrt_handler->check_collision(
                nearest_node_id, new_node,
                this->get_parameter("rrt_check_pts_num").as_int())
            )   continue;
        }

        rrt_handler->tree.push_back(new_node);
        node_tracker = new_node;

        if(rrt_handler->calcDistance(target_pt_world, new_node.x, new_node.y) < this->get_parameter("rrt_hit_dist").as_double()){
            path = rrt_handler->find_path(new_node);
            break;
        }
    }

    rrt_handler->ema_enable = this->get_parameter("rrt_ema_enable").as_bool();
    rrt_handler->ema_alpha = this->get_parameter("rrt_ema_alpha").as_double();
    std::vector<RRT_Node> local_path = rrt_handler->get_local_path(path, pure_pursuit_handler->t);

    // visualize the path
    rrt_handler->visualize_local_path(local_path);
    marker_publisher_->publish(rrt_handler->visualized_points);

    rrt_handler->high_speed = this->get_parameter("rrt_high_speed").as_double();
    rrt_handler->medium_speed = this->get_parameter("rrt_medium_speed").as_double();
    rrt_handler->low_speed = this->get_parameter("rrt_low_speed").as_double();
    rrt_handler->kp = this->get_parameter("rrt_kp").as_double();
    ackermann_msgs::msg::AckermannDriveStamped control = rrt_handler->follow_path(local_path, this->get_parameter("rrt_track_dist").as_double());
    if(control.drive.speed >= 0.0) drive_publisher_->publish(control);
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