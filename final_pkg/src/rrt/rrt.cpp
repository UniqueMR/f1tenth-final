#include "rrt/rrt.hpp"

rrtHandler::rrtHandler(std::string waypoints_path, std::string parent_frame_id){
    updated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    dataloader = std::make_unique<wayPointLoader>(waypoints_path.c_str());
    way_points = dataloader->way_points;
    updated_map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    init_map_header(parent_frame_id);
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