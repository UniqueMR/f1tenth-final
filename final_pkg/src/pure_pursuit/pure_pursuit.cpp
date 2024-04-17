#include "pure_pursuit/pure_pursuit.hpp"

purePursuitHandler::purePursuitHandler(std::string waypoints_path){
    update_params(1.5, 0.5, 2.0);
    dataloader = std::make_unique<wayPointLoader>(waypoints_path); 
    way_points = dataloader->way_points;    
}

void purePursuitHandler::update_params(double look_ahead_dist, double kp, double speed){
    this->look_ahead_dist = look_ahead_dist, this->kp = kp, this->speed = speed;
}

purePursuitHandler::~purePursuitHandler(){
    
}
