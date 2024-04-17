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

void purepursuitHandler::get_transform_stamp(std::string parent_frame_id, std::string child_frame_id, std::shared_ptr<tf2_ros::Buffer> tf_buffer_){
    try {
        t = tf_buffer_->lookupTransform(
        child_frame_id, parent_frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout << "Could not transform " << parent_frame_id.c_str() << " to " << child_frame_id.c_str() << "!"; 
        return;
    }
}

