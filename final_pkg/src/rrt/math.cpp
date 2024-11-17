#include "rrt/rrt.hpp"

double rrtHandler::calcDistance(std::vector<double> sampled_pt, double node_x, double node_y){
    double dist_x = sampled_pt[0] - node_x;
    double dist_y = sampled_pt[1] - node_y;

    return std::pow(dist_x, 2) + std::pow(dist_y, 2);
}


double rrtHandler::line_cost(RRT_Node &n1, RRT_Node &n2){
    return std::sqrt(std::pow(n1.x - n2.x, 2) + std::pow(n1.y - n2.y, 2));
}

void transformStampedToMatrix(const geometry_msgs::msg::TransformStamped &transform, float matrix[16]) {
    // Extract translation
    const auto &t = transform.transform.translation;

    // Extract rotation as quaternion
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );

    // Convert quaternion to rotation matrix
    tf2::Matrix3x3 m(q);

    // Populate the forward transformation matrix
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            matrix[i*4+j] = static_cast<float>(m[i][j]);
        }
    }

    // Set the translation part
    matrix[3] = static_cast<float>(t.x);
    matrix[7] = static_cast<float>(t.y);
    matrix[11] = static_cast<float>(t.z);

    // Set the bottom row of the homogeneous matrix
    matrix[12] = 0.0f;
    matrix[13] = 0.0f;
    matrix[14] = 0.0f;
    matrix[15] = 1.0f;
}

void tb_cuda_local_to_world(double curr_local_x, double curr_local_y, double &curr_global_x, double &curr_global_y, const geometry_msgs::msg::TransformStamped &transform) {
    float t_mat[16];

    // Generate the forward transformation matrix
    transformStampedToMatrix(transform, t_mat);

    // Apply the transformation
    curr_global_x = t_mat[0] * curr_local_x + t_mat[1] * curr_local_y + t_mat[3];
    curr_global_y = t_mat[4] * curr_local_x + t_mat[5] * curr_local_y + t_mat[7];
}