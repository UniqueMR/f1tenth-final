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

    updated_map->data.assign(updated_map->info.width * updated_map->info.height, -1);
}

void rrtHandler::init_marker(std::string parent_frame_id){
    visualized_points.header.frame_id = parent_frame_id;
    visualized_points.ns = "sampled_points";
    visualized_points.action = visualization_msgs::msg::Marker::ADD;
    visualized_points.pose.orientation.w = 1.0;

    visualized_points.id = 0;
    visualized_points.type = visualization_msgs::msg::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    visualized_points.scale.x = 0.05; // Specify the size of the point
    visualized_points.scale.y = 0.05;

    visualized_points.color.r = 1.0f; // Set the red component to full intensity
    visualized_points.color.g = 0.0f; // Set the green component to zero
    visualized_points.color.b = 0.0f; // Set the blue component to zero
    visualized_points.color.a = 1.0f; // Don't forget to set the alpha!
}

void rrtHandler::visualize_increment_path(RRT_Node path_pt){
    geometry_msgs::msg::Point p;
    p.x = path_pt.x, p.y = path_pt.y, p.z = 0;
    visualized_points.points.push_back(p);
}
