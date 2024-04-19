#include "rrt/rrt.hpp"

rrtHandler::rrtHandler(std::string waypoints_path, std::string parent_frame_id){
    updated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    dataloader = std::make_unique<wayPointLoader>(waypoints_path.c_str());
    way_points = dataloader->way_points;
    updated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    init_map_header(parent_frame_id);
    init_marker(parent_frame_id);
}

rrtHandler::~rrtHandler(){
    
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

                std::vector<int> obs_idxs = get_obs_idx(
                    curr_beam_world, 
                    bubble_offset
                );

                for(const auto& idx : obs_idxs){
                    if(idx < 0 || idx >= updated_map->info.width * updated_map->info.height)  
                        continue;                    
                    updated_map->data[idx] = 100;
                }
            }   

            clear_obs_cnt++;
            if(clear_obs_cnt > obs_clear_rate){
                updated_map->data.assign(updated_map->info.width * updated_map->info.height, -1);
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
    int nearest_node = 0;
    double nearest_dist = std::numeric_limits<double>::infinity();
    // TODO: fill in this method 
    for(unsigned int i = 0; i < tree.size(); i++)
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

double rrtHandler::calcDistance(std::vector<double> sampled_pt, double node_x, double node_y){
    double dist_x = sampled_pt[0] - node_x;
    double dist_y = sampled_pt[1] - node_y;

    return std::pow(dist_x, 2) + std::pow(dist_y, 2);
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

bool rrtHandler::check_collision(int neighbor_idx, RRT_Node new_node, int check_pts_num){
    
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

double rrtHandler::line_cost(RRT_Node &n1, RRT_Node &n2){
    return std::sqrt(std::pow(n1.x - n2.x, 2) + std::pow(n1.y - n2.y, 2));
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

int rrtHandler::link_best_neighbor(RRT_Node &new_node, std::vector<int> neighbor_indices, std::vector<bool> &neighbor_collided, int check_pts_num){
    int best_neighbor_idx = tree.size();
    for(const int neighbor_idx : neighbor_indices){
        if(check_collision(neighbor_idx, new_node, check_pts_num)){
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
    geometry_msgs::msg::PointStamped curr_pt_world, 
    geometry_msgs::msg::TransformStamped t, double look_ahead_dist){
    double target_pt_dist = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::PointStamped next_pt_local_temp, next_pt_world_temp, next_pt_world;

    geometry_msgs::msg::PointStamped curr_pt_local;
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
    return path;
}

std::vector<double> rrtHandler::follow_path(std::vector<RRT_Node> path, geometry_msgs::msg::TransformStamped t, double look_ahead_dist, double kp){
    std::vector<double> steerings;
    geometry_msgs::msg::PointStamped curr_pt_local, next_pt_world, next_pt_local;
    curr_pt_local.point.x = 0, curr_pt_local.point.y = 0;
    for(RRT_Node path_pt : path){
        next_pt_world.point.x = path_pt.x, next_pt_world.point.y = path_pt.y;
        try{
            tf2::doTransform(next_pt_world, next_pt_local, t);
        }
        catch(tf2::TransformException &ex){
            std::cout << "path world to local failed!" << std::endl;
        }
        double y = next_pt_local.point.y - curr_pt_local.point.y;
        double curvature = (2 * y) / (look_ahead_dist * look_ahead_dist);
        steerings.push_back(kp * curvature);
        curr_pt_local = next_pt_local;
    }
    return steerings;
}

void rrtHandler::init_map_header(std::string frame_id){
    updated_map->header.frame_id = frame_id;
    // Specify the layout of the map
    updated_map->info.resolution = 0.1; // each cell will represent 10cm x 10cm
    updated_map->info.width = 759;      // 10m wide
    updated_map->info.height = 844;     // 10m tall
    updated_map->info.origin.position.x = -27.7;
    updated_map->info.origin.position.y = -12.4;
    updated_map->info.origin.position.z = 0.0;
    updated_map->info.origin.orientation.x = 0.0;
    updated_map->info.origin.orientation.y = 0.0;
    updated_map->info.origin.orientation.z = 0.0;
    updated_map->info.origin.orientation.w = 1.0;

    updated_map->data.assign(updated_map->info.width * updated_map->info.height, -1);
}

void rrtHandler::init_marker(std::string parent_frame_id){
    visualized_points.header.frame_id = parent_frame_id; // Change to your frame ID
    visualized_points.ns = "sampled_points";
    visualized_points.action = visualization_msgs::msg::Marker::ADD;
    visualized_points.pose.orientation.w = 1.0;

    visualized_points.id = 0;
    visualized_points.type = visualization_msgs::msg::Marker::POINTS;

    // POINTS markers use x and y scale for width/height respectively
    visualized_points.scale.x = 0.2; // Specify the size of the point
    visualized_points.scale.y = 0.2;

    visualized_points.color.r = 1.0f; // Set the red component to full intensity
    visualized_points.color.g = 0.0f; // Set the green component to zero
    visualized_points.color.b = 0.0f; // Set the blue component to zero
    visualized_points.color.a = 1.0f; // Don't forget to set the alpha!
}