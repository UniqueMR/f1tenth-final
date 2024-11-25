cmake_minimum_required(VERSION 3.5)
project(final_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable CUDA language
enable_language(CUDA)

# Check for the availability of CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(PACKAGE 
  ament_cmake 
  ament_cmake_python
  rclcpp
  rclpy 
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  Eigen3
  tf2_ros
  tf2_geometry_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include/${PROJECT_NAME})
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Create Cpp executable with CUDA code
add_executable(executer_node 
  src/executer_node.cpp 
  src/executer.cpp 
  src/utils/csv_loader.cpp 
  src/blocking/blocking.cpp 
  src/pure_pursuit/pure_pursuit.cpp 
  src/rrt/rrt.cpp 
  src/rrt/math.cpp 
  src/rrt/visualize.cpp 
  src/rrt/debug.cpp
  src/cuda/cuda.cu  # Add CUDA file
)

ament_target_dependencies(executer_node 
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2_geometry_msgs
)
target_include_directories(executer_node PRIVATE ${CUDA_INCLUDE_DIRS})


# Link CUDA libraries
target_link_libraries(executer_node 
  ${CUDA_LIBRARIES}
)

# Install Cpp executables
install(TARGETS
  executer_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/sim_map_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/get_obs_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoints/waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoints/waypoint_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment the following line to skip copyright checks
  # set(ament_cmake_copyright_FOUND TRUE)
  # Uncomment the following line to skip cpplint
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
