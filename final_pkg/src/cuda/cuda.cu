// my_cuda_code.cu
#include <cuda_runtime.h>
#include <cstdio>

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
