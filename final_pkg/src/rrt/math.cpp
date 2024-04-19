#include "rrt/rrt.hpp"

double rrtHandler::calcDistance(std::vector<double> sampled_pt, double node_x, double node_y){
    double dist_x = sampled_pt[0] - node_x;
    double dist_y = sampled_pt[1] - node_y;

    return std::pow(dist_x, 2) + std::pow(dist_y, 2);
}


double rrtHandler::line_cost(RRT_Node &n1, RRT_Node &n2){
    return std::sqrt(std::pow(n1.x - n2.x, 2) + std::pow(n1.y - n2.y, 2));
}