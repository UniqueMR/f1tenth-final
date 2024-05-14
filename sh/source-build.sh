source /opt/ros/humble/setup.bash
source install/setup.bash
# colcon build --packages-select $1 --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfog
colcon build --packages-select $1
