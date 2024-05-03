#include <string>
#include <memory>
#include <random>
#include <algorithm>
#include "utils/csv_loader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "tf2_ros/buffer.h"

typedef struct RRT_Node {
    double x, y; // not sure
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

class rrtHandler{
public:
    rrtHandler(std::string waypoints_path, std::string parent_frame_id);
    virtual ~rrtHandler();

    // params
    bool ema_enable;
    double ema_alpha;

    double high_speed, medium_speed, low_speed;

    double kp;

    // init 
    void init_map_header(std::string frame_id);
    void init_marker(std::string parent_frame_id);
    void init_tree(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    // visualization
    visualization_msgs::msg::Marker visualized_points;
    visualization_msgs::msg::Marker visualized_target;
    int clear_obs_cnt = 0;
    bool clear_state = false;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> updated_map;
    void update_occupancy_grid(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
        double look_ahead_dist, int bubble_offset, int obs_clear_rate
    );
    std::vector<int> get_obs_idx(
        geometry_msgs::msg::PointStamped pt_world,
        double bubble_offset
    );
    void visualize_local_path(std::vector<RRT_Node> local_path_node);
    void visualize_target(std::vector<double> target);

    // transformation
    geometry_msgs::msg::TransformStamped t;
    void get_transform_stamp_L2W(
        std::string parent_frame_id, 
        std::string child_frame_id, 
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_
    );

    // workflow
    std::vector<RRT_Node> tree;
    std::vector<double> sample(double look_ahead_dist);
    bool is_collide(std::vector<double> sampled_pt);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    int nearest(std::vector<double> sampled_node_pt);
    double calcDistance(std::vector<double> sampled_pt, double node_x, double node_y);
    RRT_Node steer(int nearest_node_id, std::vector<double> sampled_node_pt, double max_expansion_dist);
    bool check_collision(int neighbor_idx, RRT_Node new_node, int check_pts_num);
    double cost(RRT_Node node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(RRT_Node node, int search_radius);
    int link_best_neighbor(RRT_Node &new_node, std::vector<int> neighbor_indices, std::vector<bool> &neighbor_collided, int check_pts_num);
    void rearrange_tree(int best_neighbor_idx, std::vector<int> neighbor_indices, std::vector<bool> neighbor_collided, RRT_Node new_node);
    std::vector<double> get_target_pt(nav_msgs::msg::Odometry::ConstSharedPtr pose_msg, geometry_msgs::msg::TransformStamped t, double look_ahead_dist);

    std::vector<RRT_Node> find_path(RRT_Node target_node);
    void ema_smoothing_local(std::vector<RRT_Node> &path, double alpha);
    std::vector<RRT_Node> get_local_path(std::vector<RRT_Node> path, geometry_msgs::msg::TransformStamped t);
    ackermann_msgs::msg::AckermannDriveStamped follow_path(std::vector<RRT_Node> local_path, double track_dist);
    std::vector<RRT_Node> path_not_found_handle(nav_msgs::msg::Odometry::ConstSharedPtr pose_msg, std::vector<double> target_pt_world);
    
private:
    std::unique_ptr<wayPointLoader> dataloader;
    std::vector<wayPoint> way_points;

    std::vector<size_t> new_obs;

    // sampling
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

};