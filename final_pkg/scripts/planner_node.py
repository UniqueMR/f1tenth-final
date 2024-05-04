#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import time

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.state_publisher_ = self.create_publisher(String, 'strategy', 10)
        self.declare_parameter("timer_period", 20.0)
        self.declare_parameter('overtake_trigger_dist_diff', 3.0)
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        obs_scan_topic = 'obs_scan'
        self.obs_scan_subscriber_ = self.create_subscription(LaserScan, obs_scan_topic, self.obs_scan_callback, 10)
        self.prev_obs_scan = np.full((1080, ), np.inf)
        self.curr_obs_scan = np.full((1080, ), np.inf)
        self.ttc_arr = [np.inf for _ in range(1080)]

        self.curr_state = 'normal'
        self.overtake_time = 0.0

    def obs_scan_callback(self, obs_scan_msg):
        self.curr_obs_scan = obs_scan_msg.ranges
        self.ttc_arr = [self.curr_obs_scan[i] - self.prev_obs_scan[i] if self.curr_obs_scan[i] != np.inf and self.prev_obs_scan[i] != np.inf else np.inf for i in range(1080)]
        self.ttc_arr = [np.abs(self.ttc_arr[i]) if self.ttc_arr[i] < 0 else 0 for i in range(1080)]
        self.max_dist_diff = max(self.ttc_arr)
        # print('max dist diff: ', self.max_dist_diff)
        if self.max_dist_diff > self.get_parameter('overtake_trigger_dist_diff').get_parameter_value().double_value and self.curr_state != 'overtake': 
            self.curr_state = 'overtake'
            self.overtake_time = time.time()

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)

        if time.time() - self.overtake_time >= 5.0 and self.curr_state == 'overtake':
            self.curr_state = 'normal'

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)

        self.prev_obs_scan = self.curr_obs_scan

    # def timer_callback(self):
    #     # this is just a dummy planner
    #     # set it to maintain a certain state for single functionality test
    #     # if self.curr_state == 'normal':
    #     #     self.curr_state = 'overtake'
    #     # elif self.curr_state == 'overtake':
    #     #     self.curr_state = 'normal'
    #     # else:
    #     #     pass
    #     self.curr_state = 'overtake'
        
    #     state_msg = String()
    #     state_msg.data = self.curr_state
    #     self.state_publisher_.publish(state_msg)

    

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()