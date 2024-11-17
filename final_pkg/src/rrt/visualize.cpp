#include "rrt/rrt.hpp"

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

    updated_map->data.assign(updated_map->info.width * updated_map->info.height, 0);


    updated_map_cuda->header.frame_id = frame_id;
    // Specify the layout of the map
    updated_map_cuda->info.resolution = 0.1; // each cell will represent 10cm x 10cm
    updated_map_cuda->info.width = 759;      // 10m wide
    updated_map_cuda->info.height = 844;     // 10m tall
    updated_map_cuda->info.origin.position.x = -27.7;
    updated_map_cuda->info.origin.position.y = -12.4;
    updated_map_cuda->info.origin.position.z = 0.0;
    updated_map_cuda->info.origin.orientation.x = 0.0;
    updated_map_cuda->info.origin.orientation.y = 0.0;
    updated_map_cuda->info.origin.orientation.z = 0.0;
    updated_map_cuda->info.origin.orientation.w = 1.0;

    updated_map_cuda->data.assign(updated_map->info.width * updated_map->info.height, 0);
}

void rrtHandler::init_marker(std::string parent_frame_id){
    // initialize header for visualized path
    visualized_points.header.frame_id = parent_frame_id;
    visualized_points.ns = "sampled_points";
    visualized_points.action = visualization_msgs::msg::Marker::ADD;
    visualized_points.pose.orientation.w = 1.0;

    visualized_points.id = 0;
    visualized_points.type = visualization_msgs::msg::Marker::LINE_STRIP;

    visualized_points.scale.x = 0.05; 
    visualized_points.scale.y = 0.05;

    visualized_points.color.r = 1.0f; 
    visualized_points.color.g = 0.0f; 
    visualized_points.color.b = 0.0f; 
    visualized_points.color.a = 1.0f; 

    // initialize header for visualized target
    visualized_target.header.frame_id = parent_frame_id;
    visualized_target.ns = "target";
    visualized_target.action = visualization_msgs::msg::Marker::ADD;
    visualized_target.pose.orientation.w = 1.0;

    visualized_target.id = 1;
    visualized_target.type = visualization_msgs::msg::Marker::POINTS;

    visualized_target.scale.x = 0.2; 
    visualized_target.scale.y = 0.2;

    visualized_target.color.r = 0.0f; 
    visualized_target.color.g = 1.0f; 
    visualized_target.color.b = 0.0f; 
    visualized_target.color.a = 1.0f; 
}

void rrtHandler::visualize_local_path(std::vector<RRT_Node> local_path_node){
    visualized_points.points.clear();
    for(const RRT_Node node : local_path_node){
        geometry_msgs::msg::PointStamped pt_local, pt_world;
        pt_local.point.x = node.x, pt_local.point.y = node.y;
        try{
            tf2::doTransform(pt_local, pt_world, t);
        }
        catch(const tf2::TransformException &ex){
            std::cout << "transform visualized point from local to world failed!" << std::endl;
        }
        geometry_msgs::msg::Point p;
        p.x = pt_world.point.x, p.y = pt_world.point.y, p.z = 0.0;
        visualized_points.points.push_back(p);
    }
    return;
}

void rrtHandler::visualize_target(std::vector<double> target){
    visualized_target.points.clear();
    geometry_msgs::msg::Point p;
    p.x = target[0], p.y = target[1], p.z = 0.0;
    visualized_target.points.push_back(p);
}

