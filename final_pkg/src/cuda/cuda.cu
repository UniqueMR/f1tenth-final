// my_cuda_code.cu
#include <cuda_runtime.h>
#include <cstdio>
#include "rrt/rrt.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


#define ROUND_UP_TO_NEAREST(M, N) (((M) + (N)-1) / (N))

// Simple kernel that prints "Hello, World!" from the device
__global__ void hello_world_kernel() {
    printf("Hello, World! from CUDA kernel\n");
}

// Wrapper function to be called from C++
extern "C" void launch_hello_world_kernel() {
    // Launch the kernel with one block and one thread
    hello_world_kernel<<<1, 1>>>();

    // Synchronize to ensure kernel completion
    cudaDeviceSynchronize();
}

__global__ void check_collision_kernel(double pta_x, double pta_y, double ptb_x, double ptb_y, 
                                       int check_pts_num, double resolution, 
                                       double origin_x, double origin_y, int width, 
                                       int *map_data, bool *collision_flag) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i < check_pts_num) {
        // Calculate the increment step for each thread
        double x_incre = (ptb_x - pta_x) / check_pts_num;
        double y_incre = (ptb_y - pta_y) / check_pts_num;

        // Calculate the sampled point coordinates
        double sampled_pt_x = pta_x + i * x_incre;
        double sampled_pt_y = pta_y + i * y_incre;

        // Map the point to grid indices
        int idx_x = static_cast<int>((sampled_pt_x - origin_x) / resolution);
        int idx_y = static_cast<int>((sampled_pt_y - origin_y) / resolution);
        int idx = idx_y * width + idx_x;

        // Check for collision and set the flag
        if (idx_x >= 0 && idx_y >= 0 && idx < width * width) {
            if (map_data[idx] == 100) {
                *collision_flag = true;
            }
        }
    }
}

extern "C" bool check_collision_cuda(double pta_x, double pta_y, double ptb_x, double ptb_y, 
                                 int check_pts_num, double origin_x, double origin_y, 
                                 double resolution, int width, int *map_data) {
    // Device pointers
    int *d_map_data;
    bool *d_collision_flag, h_collision_flag = false;

    // Allocate memory on the device
    cudaMalloc(&d_map_data, width * width * sizeof(int));
    cudaMalloc(&d_collision_flag, sizeof(bool));

    // Copy data to device
    cudaMemcpy(d_map_data, map_data, width * width * sizeof(int), cudaMemcpyHostToDevice);
    cudaMemcpy(d_collision_flag, &h_collision_flag, sizeof(bool), cudaMemcpyHostToDevice);

    // Define grid and block size
    int blockSize = 256;
    int numBlocks = (check_pts_num + blockSize - 1) / blockSize;

    // Launch the kernel
    check_collision_kernel<<<numBlocks, blockSize>>>(pta_x, pta_y, ptb_x, ptb_y, 
                                                     check_pts_num, resolution, 
                                                     origin_x, origin_y, width, 
                                                     d_map_data, d_collision_flag);

    // Copy the result back to host
    cudaMemcpy(&h_collision_flag, d_collision_flag, sizeof(bool), cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree(d_map_data);
    cudaFree(d_collision_flag);

    return h_collision_flag;
}

const uint ranges_sz = 1080;
const uint updated_map_width = 759;
const uint updated_map_height = 844;
const double updated_map_resolution = 0.1;
const double updated_map_origin_x = -27.7;
const double updated_map_origin_y = -12.4;
const double scan_ang_min = -2.35;
const double scan_ang_increment = 0.00435185;

__global__ void update_occupancy_grid_kernel(float* ranges_arr, uint8_t* updated_map_arr, float* t_mat, double look_ahead_dist, int bubble_offset){
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if(idx < ranges_sz){
        double curr_dist = ranges_arr[idx];
        double curr_ang = scan_ang_min + idx * scan_ang_increment;

        if(!isnan(curr_dist) && !isinf(curr_dist)){
            double curr_x = curr_dist * cosf(curr_ang);
            double curr_y = curr_dist * sinf(curr_ang);
            
            // printf("Thread %d: curr_dist = %f, curr_ang = %f, curr_x = %f, curr_y = %f, look_ahead_dist = %f\n", idx, curr_dist, curr_ang, curr_x, curr_y, look_ahead_dist);
            if(fabs(curr_x) < look_ahead_dist && fabs(curr_y) < look_ahead_dist){

                double curr_global_x = t_mat[0] * curr_x + t_mat[1] * curr_y + t_mat[3];
                double curr_global_y = t_mat[4] * curr_x + t_mat[5] * curr_y + t_mat[7];

                int base_idx_x = static_cast<int>((curr_global_x - updated_map_origin_x) / updated_map_resolution);
                int base_idx_y = static_cast<int>((curr_global_y - updated_map_origin_y) / updated_map_resolution);

                for(int i = base_idx_x - bubble_offset; i < base_idx_x + bubble_offset; i++)
                    for(int j = base_idx_y - bubble_offset; j < base_idx_y + bubble_offset; j++)
                        if(j * updated_map_width + i > 0 && j * updated_map_width + i < updated_map_width * updated_map_height)
                            updated_map_arr[j * updated_map_width + i] = 100;
            }       
        }
    }
}

extern "C" void update_occupancy_grid_cuda(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg,
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> updated_map,
    geometry_msgs::msg::TransformStamped& transform,
    double look_ahead_dist, int bubble_offset
){
    float* ranges_arr;
    uint8_t* updated_map_arr;
    cudaMalloc(&ranges_arr, ranges_sz * sizeof(float));
    cudaMalloc(&updated_map_arr, updated_map_height * updated_map_width);

    cudaMemcpy(ranges_arr, scan_msg->ranges.data(), ranges_sz * sizeof(float), cudaMemcpyHostToDevice);


    cudaMemset(updated_map_arr, 0, updated_map_height * updated_map_width * sizeof(uint8_t)); 
    
    dim3 gridDim(ROUND_UP_TO_NEAREST(ranges_sz, 256));
    dim3 blockDim(256);

    float t_mat[16];
    transformStampedToMatrix(transform, t_mat);
    float* d_t_mat;

    cudaMalloc(&d_t_mat, 4 * 4 * sizeof(float));
    cudaMemcpy(d_t_mat, t_mat, 4 * 4 * sizeof(float), cudaMemcpyHostToDevice);

    update_occupancy_grid_kernel<<<gridDim, blockDim>>>(ranges_arr, updated_map_arr, d_t_mat, look_ahead_dist, bubble_offset);

    cudaMemcpy(updated_map->data.data(), updated_map_arr, updated_map_width * updated_map_height * sizeof(uint8_t), cudaMemcpyDeviceToHost);

    cudaFree(ranges_arr);
    cudaFree(updated_map_arr);
    cudaFree(d_t_mat);
    
}
