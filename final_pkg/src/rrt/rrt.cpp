#include "rrt/rrt.hpp"

rrtHandler::rrtHandler(std::string waypoints_path, std::string parent_frame_id, int _check_pts_num){
    updated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    updated_map_cuda = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    dataloader = std::make_unique<wayPointLoader>(waypoints_path.c_str());
    way_points = dataloader->way_points;
    init_map_header(parent_frame_id);
    init_marker(parent_frame_id);
    new_obs = {};
    new_obs.reserve(2000);
    logfile.open("transformation.txt", std::ios::app);
    if(!logfile.is_open())  std::cerr << "failed to open logfile for transformation" << std::endl; 
    cudaMalloc(&ranges_arr, ranges_sz * sizeof(float));
    cudaMalloc(&updated_map_arr, updated_map_height * updated_map_width);
    cudaMalloc(&d_t_mat, 4 * 4 * sizeof(float));
    this->check_pts_num = _check_pts_num;
    this->init_thrust();
}

rrtHandler::~rrtHandler(){
    logfile.close();
    cudaFree(ranges_arr);
    cudaFree(updated_map_arr);
    cudaFree(d_t_mat);
}

void rrtHandler::init_tree(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg){
    tree.clear();
    RRT_Node curr_vehicle_world;
    curr_vehicle_world.x = pose_msg->pose.pose.position.x;
    curr_vehicle_world.y = pose_msg->pose.pose.position.y;
    curr_vehicle_world.cost = 0.0;
    curr_vehicle_world.parent = -1;
    curr_vehicle_world.is_root = true;
    tree.push_back(curr_vehicle_world);
}

void rrtHandler::get_transform_stamp_L2W(
    std::string parent_frame_id, 
    std::string child_frame_id, 
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_
){
    try {
        t = tf_buffer_->lookupTransform(
        parent_frame_id, child_frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout << "Could not transform " << parent_frame_id.c_str() << " to " << child_frame_id.c_str() << "!"; 
        return;
    }
}

constexpr double EPSILON = 1e-6;

bool areTransformsDifferent(const geometry_msgs::msg::TransformStamped &t1,
                            const geometry_msgs::msg::TransformStamped &t2) {
    // Compare translations
    const auto &trans1 = t1.transform.translation;
    const auto &trans2 = t2.transform.translation;

    if (std::fabs(trans1.x - trans2.x) > EPSILON ||
        std::fabs(trans1.y - trans2.y) > EPSILON ||
        std::fabs(trans1.z - trans2.z) > EPSILON) {
        return true; // Translation is different
    }

    // Compare rotations (quaternion)
    const auto &rot1 = t1.transform.rotation;
    const auto &rot2 = t2.transform.rotation;

    if (std::fabs(rot1.x - rot2.x) > EPSILON ||
        std::fabs(rot1.y - rot2.y) > EPSILON ||
        std::fabs(rot1.z - rot2.z) > EPSILON ||
        std::fabs(rot1.w - rot2.w) > EPSILON) {
        return true; // Rotation is different
    }

    // If no differences are found, they are considered the same
    return false;
}

void rrtHandler::update_occupancy_grid(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    double look_ahead_dist, int bubble_offset, int obs_clear_rate){
    for(unsigned int i = 0; i < scan_msg->ranges.size(); i++){
        double curr_dist = scan_msg->ranges[i];
        double curr_ang = scan_msg->angle_min + i * scan_msg->angle_increment;

        // check if the lidar ranges are nan or inf
        if(std::isnan(curr_dist) || std::isinf(curr_dist))  continue;

        // get the position of the obstacle in the vehicle frame
        double curr_x = curr_dist * std::cos(curr_ang), curr_y = curr_dist * std::sin(curr_ang);

        if(curr_x > look_ahead_dist || curr_y > look_ahead_dist)  continue;
    
        geometry_msgs::msg::PointStamped curr_beam_local, curr_beam_world; 
        curr_beam_local.point.x = curr_x, curr_beam_local.point.y = curr_y;

        try {
        tf2::doTransform(curr_beam_local, curr_beam_world, t);
        } catch (const tf2::TransformException &ex) {
        std::cout << "beam transformation failed" << std::endl;
        }

        // double curr_beam_world_cuda_x, curr_beam_world_cuda_y; 
        // tb_cuda_local_to_world(curr_x, curr_y, curr_beam_world_cuda_x, curr_beam_world_cuda_y, t);

        // logfile << curr_beam_local.point.x << ", " << curr_beam_local.point.y << ", " 
        // << curr_beam_world.point.x << ", " << curr_beam_world.point.y << ", " 
        //     << curr_beam_world_cuda_x << ", " << curr_beam_world_cuda_y << std::endl; 

        // logfile_lines++;

        // if(logfile_lines == 100) logfile.close();

        // if(t_init){
        //     prev_t = t;
        //     t_init = false;
        // }

        // if(areTransformsDifferent(t, prev_t)){
        //     logfile.open("transformation.txt", std::ios::app);
        //     logfile_lines = 0;
        //     prev_t = t;
        // }   

        std::vector<int> obs_idxs = get_obs_idx(
            curr_beam_world, 
            bubble_offset
        );

        for(const auto& idx : obs_idxs){
            if(idx < 0 || idx >= updated_map->info.width * updated_map->info.height)  
                continue;                    
            if(updated_map->data[idx] != 100){
                updated_map->data[idx] = 100;
                new_obs.emplace_back(idx);
            }
        }
    }

    clear_state = false;   

    clear_obs_cnt++;
    if(clear_obs_cnt > obs_clear_rate){
        clear_state = true;
        for(const auto idx : new_obs)   updated_map->data[idx] = 0;
        new_obs.clear();
        clear_obs_cnt = 0;
    }
    
}

std::vector<double> rrtHandler::sample(double look_ahead_dist) {
    std::vector<double> sampled_point;

    std::uniform_real_distribution<>::param_type x_param(0, look_ahead_dist);
    std::uniform_real_distribution<>::param_type y_param(-look_ahead_dist, look_ahead_dist);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::msg::PointStamped sampled_pt_local, sampled_pt_world;
    sampled_pt_local.point.x = x_dist(gen);
    sampled_pt_local.point.y = y_dist(gen);

    try {
    tf2::doTransform(sampled_pt_local, sampled_pt_world, t);
    } catch (const tf2::TransformException &ex) {
    std::cout << "sampled point transformation failed" << std::endl;
    }

    sampled_point.push_back(sampled_pt_world.point.x);
    sampled_point.push_back(sampled_pt_world.point.y);

    // std::cout << "local: " << sampled_pt_local.point.x << ", " << sampled_pt_local.point.y << std::endl;
    // std::cout << "world: " << sampled_pt_world.point.x << ", " << sampled_pt_world.point.y << std::endl;     
    return sampled_point;
}

bool rrtHandler::is_collide(std::vector<double> sampled_pt){
    int idx_x = static_cast<int>((sampled_pt[0] - updated_map->info.origin.position.x) / updated_map->info.resolution);
    int idx_y = static_cast<int>((sampled_pt[1] - updated_map->info.origin.position.y) / updated_map->info.resolution);
    int idx = idx_y * updated_map->info.width + idx_x;

    return updated_map->data[idx] == 100;
}

int rrtHandler::nearest(std::vector<double> sampled_node_pt){
    int nearest_node = -1;
    double nearest_dist = std::numeric_limits<double>::infinity();
    // TODO: fill in this method 
    for(int i = 0; i < tree.size(); i++)
    {   
        double distance = calcDistance(sampled_node_pt, tree[i].x, tree[i].y); 
        if(distance < nearest_dist)
        {
            nearest_node = i;
            nearest_dist = distance;
        }
    }   

    return nearest_node;
}

std::vector<int> rrtHandler::get_obs_idx(
    geometry_msgs::msg::PointStamped pt_world,
    double bubble_offset){    
    std::vector<int> obs_idxs;
    int base_idx_x = static_cast<int>((pt_world.point.x - updated_map->info.origin.position.x) / updated_map->info.resolution);
    int base_idx_y = static_cast<int>((pt_world.point.y - updated_map->info.origin.position.y) / updated_map->info.resolution);

    for(int idx = base_idx_x - bubble_offset; idx < base_idx_x + bubble_offset; idx++){
        for(int idy = base_idx_y - bubble_offset; idy < base_idx_y + bubble_offset; idy++)
            obs_idxs.push_back(idy * updated_map->info.width + idx);
    }

    return obs_idxs;
}

RRT_Node rrtHandler::steer(int nearest_node_id, std::vector<double> sampled_node_pt, double max_expansion_dist){
    RRT_Node new_node;
    std::vector<double> vec_node_sample = {sampled_node_pt[0] - tree[nearest_node_id].x, sampled_node_pt[1] - tree[nearest_node_id].y};
    double dist_node_sample = std::sqrt(std::pow(vec_node_sample[0], 2) + std::pow(vec_node_sample[1], 2));
    std::vector<double> norm_vec = {vec_node_sample[0] / dist_node_sample, vec_node_sample[1] / dist_node_sample};

    new_node.x = tree[nearest_node_id].x + ((dist_node_sample < max_expansion_dist) ? norm_vec[0] * dist_node_sample : norm_vec[0] * max_expansion_dist);
    new_node.y = tree[nearest_node_id].y + ((dist_node_sample < max_expansion_dist) ? norm_vec[1] * dist_node_sample : norm_vec[1] * max_expansion_dist);

    new_node.parent = nearest_node_id;
    new_node.is_root = false;

    return new_node;
}

bool rrtHandler::check_collision(int neighbor_idx, RRT_Node new_node){
    
    // create a new temp node along the line between nearest node and new node 
    double x_incre = (new_node.x - tree[neighbor_idx].x) / check_pts_num;
    double y_incre = (new_node.y - tree[neighbor_idx].y) / check_pts_num;

    for(int i = 0; i < check_pts_num; i++)
    {
        double sampled_pt_x = tree[neighbor_idx].x + i * x_incre;
        double sampled_pt_y = tree[neighbor_idx].y + i * y_incre;

        std::vector<double> sampled_pt;
        sampled_pt.push_back(sampled_pt_x);
        sampled_pt.push_back(sampled_pt_y);

        if(is_collide(sampled_pt))    return true;
    }

    return false;
}

double rrtHandler::cost(RRT_Node node){
    double parent_cost = tree[node.parent].cost;
    double curr_cost = line_cost(tree[node.parent], node);
    return parent_cost + curr_cost;
}

std::vector<int> rrtHandler::near(RRT_Node node, int search_radius) {
    std::vector<int> neighborhood;
    for(int i = 0; i < tree.size(); i++){
        if(std::sqrt(std::pow(tree[i].x - node.x, 2) + std::pow(tree[i].y - node.y, 2)) < search_radius)
            neighborhood.push_back(i);
    }
    return neighborhood;
}

int rrtHandler::link_best_neighbor(RRT_Node &new_node, std::vector<int> neighbor_indices, std::vector<bool> &neighbor_collided){
    int best_neighbor_idx = -1;
    for(const int neighbor_idx : neighbor_indices){
        if(check_collision(neighbor_idx, new_node)){
            neighbor_collided.push_back(true);
            continue;
        }
        
        neighbor_collided.push_back(false);

        double curr_cost = tree[neighbor_idx].cost + line_cost(tree[neighbor_idx], new_node);

        if(curr_cost < new_node.cost){
            new_node.parent = neighbor_idx;
            new_node.cost = curr_cost;
            best_neighbor_idx = neighbor_idx;
        }
    }
    return best_neighbor_idx;
}

void rrtHandler::rearrange_tree(int best_neighbor_idx, std::vector<int> neighbor_indices, std::vector<bool> neighbor_collided, RRT_Node new_node){
    if(best_neighbor_idx == tree.size())    return;
    for(int i = 0; i < neighbor_indices.size(); i++){
        if(neighbor_collided[i] || i == best_neighbor_idx)  continue;
        double potential_cost = new_node.cost + line_cost(new_node, tree[neighbor_indices[i]]);
        if(potential_cost < tree[neighbor_indices[i]].cost){
            tree[neighbor_indices[i]].parent = tree.size();
            tree[neighbor_indices[i]].cost = potential_cost;
        }
    }
}

std::vector<double> rrtHandler::get_target_pt(
    nav_msgs::msg::Odometry::ConstSharedPtr pose_msg, 
    geometry_msgs::msg::TransformStamped t, double look_ahead_dist){
    double target_pt_dist = std::numeric_limits<double>::infinity();

    geometry_msgs::msg::PointStamped curr_pt_world, curr_pt_local, next_pt_local_temp, next_pt_world_temp, next_pt_world;
    curr_pt_world.point.x = pose_msg->pose.pose.position.x;
    curr_pt_world.point.y = pose_msg->pose.pose.position.y;
    try {
    tf2::doTransform(curr_pt_world, curr_pt_local, t);
    } catch (const tf2::TransformException &ex) {
    std::cout << "current point transformation failed" << std::endl;
    }

    // search in way_points for potential target
    for(auto way_point : way_points){
        double curr_pt_dist = std::sqrt(std::pow((way_point.x - curr_pt_world.point.x), 2) + std::pow((way_point.y - curr_pt_world.point.y), 2));
        // pass points inside lookahead distance
        if(curr_pt_dist < look_ahead_dist)   continue;
        if(curr_pt_dist < target_pt_dist){
            //transform potential target to local frame
            try {
            next_pt_world_temp.point.x = way_point.x;
            next_pt_world_temp.point.y = way_point.y;
            tf2::doTransform(next_pt_world_temp, next_pt_local_temp, t);
            } catch (const tf2::TransformException &ex) {
            std::cout << "next point transformation failed" << std::endl;
            }
            
            // pass way_point that falls behind
            if(next_pt_local_temp.point.x <= curr_pt_local.point.x)   continue;

            // update selected next point in world frame
            target_pt_dist = curr_pt_dist;
            next_pt_world.point.x = curr_pt_world.point.x + (way_point.x - curr_pt_world.point.x) * look_ahead_dist / curr_pt_dist;
            next_pt_world.point.y = curr_pt_world.point.y + (way_point.y - curr_pt_world.point.y) * look_ahead_dist / curr_pt_dist;
        }
    }

    std::vector<double> next_pt_world_vec;
    next_pt_world_vec.push_back(next_pt_world.point.x);
    next_pt_world_vec.push_back(next_pt_world.point.y);
    return next_pt_world_vec;
}

std::vector<RRT_Node> rrtHandler::find_path(RRT_Node target_node){
    RRT_Node searched_node = target_node;
    std::vector<RRT_Node> path;
    while(searched_node.is_root == false){
        path.push_back(searched_node);
        searched_node = tree[searched_node.parent];
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void rrtHandler::ema_smoothing_local(std::vector<RRT_Node> &path, double alpha){
    for(int i = 1; i < path.size(); i++){
        path[i].y = alpha * path[i].y + (1 - alpha) * path[i - 1].y;
    }
    return;
}

std::vector<RRT_Node> rrtHandler::get_local_path(std::vector<RRT_Node> path, geometry_msgs::msg::TransformStamped t){
    // transform path from world frame to local frame
    std::vector<RRT_Node> local_path;
    RRT_Node curr_node_local;
    curr_node_local.x =0, curr_node_local.y = 0;
    local_path.push_back(curr_node_local);
    for(RRT_Node path_pt : path){
        geometry_msgs::msg::PointStamped pt_world, pt_local;
        pt_world.point.x = path_pt.x, pt_world.point.y = path_pt.y;
        try{
            tf2::doTransform(pt_world, pt_local, t);
        }
        catch(tf2::TransformException &ex){
            std::cout << "path world to local failed!" << std::endl;
        }
        RRT_Node node_local;
        node_local.x = pt_local.point.x, node_local.y = pt_local.point.y;
        local_path.push_back(node_local);
    }

    // filter path in local frame
    if(ema_enable)  ema_smoothing_local(local_path, ema_alpha);

    return local_path;
}

ackermann_msgs::msg::AckermannDriveStamped rrtHandler::follow_path(std::vector<RRT_Node> local_path, double track_dist){
    // output steering & speed
    double min_diff = std::numeric_limits<double>::max();
    RRT_Node target_node;

    // search for the target node to pursuit
    for(const RRT_Node node : local_path){
        double curr_diff = track_dist - std::sqrt(std::pow(node.x, 2) + std::pow(node.y, 2));
        if(curr_diff < min_diff){
            min_diff = curr_diff;
            target_node.x = node.x, target_node.y = node.y;
        }
    }

    // generate the steering and speed to pursuit the target node
    ackermann_msgs::msg::AckermannDriveStamped control;
    if(target_node.x < 0){   
        control.drive.steering_angle = 0.0;
        control.drive.speed = -1.0;
        return control;
    }

    double y = target_node.y * track_dist / std::sqrt(std::pow(target_node.x, 2) + std::pow(target_node.y, 2));
    double steering = 2 * kp * y / std::pow(track_dist, 2);
    control.drive.steering_angle = (steering > 0.4) ? 0.4 : ((steering < -0.4) ? -0.4 : steering);
    control.drive.speed = (steering < 0.1 && steering > -0.1) ? high_speed : ((steering > 0.2 || steering < -0.2) ? low_speed : medium_speed);
    // control.drive.speed = 0.0;
    return control;
}

std::vector<RRT_Node> rrtHandler::path_not_found_handle(nav_msgs::msg::Odometry::ConstSharedPtr pose_msg, std::vector<double> target_pt_world){
    RRT_Node curr_vehicle_world, target;

    curr_vehicle_world.x = pose_msg->pose.pose.position.x;
    curr_vehicle_world.y = pose_msg->pose.pose.position.y;
    curr_vehicle_world.cost = 0.0;
    curr_vehicle_world.parent = -1;
    curr_vehicle_world.is_root = true;

    target.x = target_pt_world[0];
    target.y = target_pt_world[1];
    target.cost = 0.0;
    target.parent = 0;
    target.is_root = false;
    
    std::vector<RRT_Node> path;

    path.push_back(curr_vehicle_world);
    path.push_back(target);

    return path;
}
