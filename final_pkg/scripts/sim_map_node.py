#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np

import gym

class simMap(Node):
    def __init__(self):
        super().__init__("sim_map")
        self.declare_parameter('sim_map_path', '')
        self.declare_parameter('sim_map_img_ext', '.png')
        self.declare_parameter('sim_num_agents', 1)
        self.declare_parameter('online', False)

        pose_topic = "pf/pose/odom" if self.get_parameter('online').get_parameter_value().bool_value else "ego_racecar/odom"
        print(pose_topic)

        # create publisher and subscriber  
        self.odom_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, 10)
        self.sim_scan_publisher_ = self.create_publisher(String, 'sim_scan', 10)

        # env backend
        self.sx, self.sy, self.stheta = 0, 0, 0
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('sim_map_path').value,
                            map_ext=self.get_parameter('sim_map_img_ext').value,
                            num_agents=self.get_parameter('sim_num_agents').get_parameter_value().integer_value)

        self.obs, _, self.done, _ = self.env.reset(np.array([[self.sx, self.sy, self.stheta]]))

    def pose_callback(self, pose_msg):
        self.sx = pose_msg.pose.pose.position.x
        self.sy = pose_msg.pose.pose.position.y
        self.obs, _, self.done, _ = self.env.reset(np.array([[self.sx, self.sy, self.stheta]]))
        
        print('curr pos: ( %f, %f)'% (self.sx, self.sy))
        print('obs: ', self.obs['scans'][0].shape)

def main(args=None):
    rclpy.init(args=args)
    sim_map = simMap()
    rclpy.spin(sim_map)
    sim_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()