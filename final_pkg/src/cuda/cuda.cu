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
