#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np 
from sensor_msgs.msg import LaserScan

class getObsNode(Node):
    def __init__(self):
        super().__init__('get_obs_node')

        self.declare_parameter("online", False)
        
        # create subscriber and publisher
        real_scan_topic = 'scan'
        sim_scan_topic = 'sim_scan'
        obs_scan_topic = 'obs_scan'

        self.child_frame_id = 'laser' if self.get_parameter('online').get_parameter_value().bool_value else 'ego_racecar/base_link'

        self.real_scan_ranges = []
        self.sim_scan_ranges = []
        self.obs_scan_ranges = np.zeros(1080)

        self.real_scan_subscriber_ = self.create_subscription(LaserScan, real_scan_topic, self.real_scan_callback, 10)
        self.sim_scan_subscriber_ =self.create_subscription(LaserScan, sim_scan_topic, self.sim_scan_callback, 10)

        self.obs_scan_msg = LaserScan()
        self.init_scan_header()
        self.obs_scan_publisher_ = self.create_publisher(LaserScan, obs_scan_topic, 10)
    
    def real_scan_callback(self, scan_msg):
        self.real_scan_ranges = scan_msg.ranges

        if len(self.real_scan_ranges) == 0 or len(self.sim_scan_ranges) == 0:
            return

        for i in range(len(self.real_scan_ranges)):
            if np.isnan(self.real_scan_ranges[i]) or np.isinf(self.real_scan_ranges[i]):
                self.real_scan_ranges[i] = 3.0

            if np.isnan(self.sim_scan_ranges[i]) or np.isinf(self.sim_scan_ranges[i]):
                self.sim_scan_ranges[i] = 3.0

            self.obs_scan_ranges[i] = self.real_scan_ranges[i] if abs(self.real_scan_ranges[i] - self.sim_scan_ranges[i]) > 0.2 else np.inf

        self.obs_scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.obs_scan_msg.ranges = self.obs_scan_ranges.tolist()
        self.obs_scan_publisher_.publish(self.obs_scan_msg)

    def sim_scan_callback(self, scan_msg):
        self.sim_scan_ranges = scan_msg.ranges

    def init_scan_header(self):
        self.obs_scan_msg.header.frame_id = self.child_frame_id
        self.obs_scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.obs_scan_msg.angle_min = -2.3499999046325684
        self.obs_scan_msg.angle_max = 2.3499999046325684
        self.obs_scan_msg.angle_increment = 0.004351851996034384
        self.obs_scan_msg.range_min = 0.0
        self.obs_scan_msg.range_max = 30.0


def main(args=None):
    rclpy.init(args=args)
    get_obs_node = getObsNode()
    rclpy.spin(get_obs_node)
    get_obs_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()