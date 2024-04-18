#include <string>
#include <memory>
#include <random>
#include "utils/csv_loader.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
    // init 
    void init_map_header(std::string frame_id);
    void init_marker( void ); 
    void init_tree(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);

    // visualization
    int clear_obs_cnt = 0;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> updated_map;
    void update_occupancy_grid(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg
    );
    std::vector<int> get_obs_idx(
        geometry_msgs::msg::PointStamped pt_world,
        double bubble_offset
    );

    // transformation
    geometry_msgs::msg::TransformStamped t;
    void get_transform_stamp_L2W(
        std::string parent_frame_id, 
        std::string child_frame_id, 
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_
    );

    // workflow
    std::vector<double> sample(double look_ahead_dist);
    bool is_collide(std::vector<double> sampled_pt);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    

private:  
    std::unique_ptr<wayPointLoader> dataloader;
    std::vector<wayPoint> way_points;
    std::vector<RRT_Node> tree;

    // sampling
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

};