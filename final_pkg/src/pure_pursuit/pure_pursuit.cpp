#include "pure_pursuit/pure_pursuit.hpp"

purepursuitHandler::purepursuitHandler(std::string waypoints_path){
    update_params(1.5, 0.5, 0.0, 0.0);
    dataloader = std::make_unique<wayPointLoader>(waypoints_path); 
    way_points = dataloader->way_points;
}

void purepursuitHandler::update_params(double look_ahead_dist, double kp, double x, double y){
    this->look_ahead_dist = look_ahead_dist, this->kp = kp;
    curr_pt_world.point.x = x, curr_pt_world.point.y = y;
}

purepursuitHandler::~purepursuitHandler(){
    
}

void purepursuitHandler::get_transform_stamp_W2L(std::string parent_frame_id, std::string child_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer_){
    try {
        t = tf_buffer_->lookupTransform(
        child_frame_id, parent_frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout << "Could not transform " << parent_frame_id.c_str() << " to " << child_frame_id.c_str() << "!"; 
        return;
    }
}

void purepursuitHandler::get_lookahead_pt(){

    double target_pt_dist = std::numeric_limits<double>::infinity();
    geometry_msgs::msg::PointStamped next_pt_world_temp;
    geometry_msgs::msg::PointStamped next_pt_local_temp;

    // get current point in local frame in the beginning
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

    // transform final next point to local frame
    try {
    tf2::doTransform(next_pt_world, next_pt_local, t);
    } catch (const tf2::TransformException &ex) {
    std::cout << "next point transformation failed" << std::endl;
    }
    return;
}

double purepursuitHandler::gen_steer_ang(){
    double y = next_pt_local.point.y - curr_pt_local.point.y;
    double curvature = (2 * y) / (look_ahead_dist * look_ahead_dist);
    return kp * curvature;
}
