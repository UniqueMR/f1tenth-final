source install/setup.bash
if [ "$1" == "pure_pursuit" ]; then
	ros2 launch pure_pursuit pure_pursuit_launch.py
elif [ "$1" == "waypoint_visualizer" ]; then
	ros2 run pure_pursuit waypoint_visualizer.py
elif [ "$1" == "rrt" ]; then
	ros2 launch lab6_pkg rrt_launch.py
elif [ "$1" == "mpc" ]; then
	ros2 run mpc mpc_node.py
elif [ "$1" == "final" ]; then
	chmod +x /home/runlong/sim_ws/install/final_pkg/lib/final_pkg/planner_node.py
	ros2 launch final_pkg final_launch.py
elif [ "$1" == "virtual_scan" ]; then
	chmod +x /home/runlong/sim_ws/install/final_pkg/lib/final_pkg/sim_map_node.py
	chmod +x /home/runlong/sim_ws/install/final_pkg/lib/final_pkg/get_obs_node.py
	ros2 launch final_pkg scan_test_launch.py
elif [ "$1" == "vis_path_pts" ]; then	
	chmod +x /home/runlong/sim_ws/install/final_pkg/lib/final_pkg/waypoint_visualizer.py
	ros2 run final_pkg waypoint_visualizer.py
elif [ "$1" == "opponent" ]; then	
	ros2 launch final_pkg oppo_launch.py
else
	echo "usage: source source-launch.sh <node-name>"
fi
