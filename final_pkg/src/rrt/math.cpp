#include "rrt/rrt.hpp"

double rrtHandler::calcDistance(std::vector<double> sampled_pt, double node_x, double node_y){
    double dist_x = sampled_pt[0] - node_x;
    double dist_y = sampled_pt[1] - node_y;

    return std::pow(dist_x, 2) + std::pow(dist_y, 2);
}


double rrtHandler::line_cost(RRT_Node &n1, RRT_Node &n2){
    return std::sqrt(std::pow(n1.x - n2.x, 2) + std::pow(n1.y - n2.y, 2));
}

void transformStampedToInverseMatrix(geometry_msgs::msg::TransformStamped& transform, float inverse_matrix[4][4]) {
    // Extract translation
    const auto& t = transform.transform.translation;
    tf2::Quaternion q(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w
    );
    tf2::Matrix3x3 m(q);

    // Rotation part of the inverse matrix (transpose of rotation matrix)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inverse_matrix[i][j] = static_cast<float>(m[j][i]);  // Transpose the rotation part
        }
    }

    // Translation part of the inverse matrix
    inverse_matrix[0][3] = -(inverse_matrix[0][0] * t.x + inverse_matrix[0][1] * t.y + inverse_matrix[0][2] * t.z);
    inverse_matrix[1][3] = -(inverse_matrix[1][0] * t.x + inverse_matrix[1][1] * t.y + inverse_matrix[1][2] * t.z);
    inverse_matrix[2][3] = -(inverse_matrix[2][0] * t.x + inverse_matrix[2][1] * t.y + inverse_matrix[2][2] * t.z);
    inverse_matrix[3][0] = 0.0f;
    inverse_matrix[3][1] = 0.0f;
    inverse_matrix[3][2] = 0.0f;
    inverse_matrix[3][3] = 1.0f;
}

void tb_cuda_local_to_world(double curr_local_x, double curr_local_y, double &curr_global_x, double &curr_global_y, geometry_msgs::msg::TransformStamped& transform){
    float t_mat[4][4];

    transformStampedToInverseMatrix(transform, t_mat);

    curr_global_x = t_mat[0][0] * curr_local_x + t_mat[0][1] * curr_local_y + t_mat[0][3];
    curr_global_y = t_mat[1][0] * curr_local_x + t_mat[1][1] * curr_local_y + t_mat[1][3];
}
