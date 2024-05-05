#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.state_publisher_ = self.create_publisher(String, 'strategy', 10)
        self.declare_parameter("timer_period", 20.0)
        self.declare_parameter('overtake_trigger_dist_diff', 3.0)
        self.declare_parameter('overtake_trigger_steering_ang', 1.5)
        self.declare_parameter('brake_dist_threshold', 0.5)
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        obs_scan_topic = 'obs_scan'
        drive_topic = 'drive'
        # self.obs_scan_subscriber_ = self.create_subscription(LaserScan, obs_scan_topic, self.obs_scan_callback, 10)
        self.drive_subscriber_ = self.create_subscription(AckermannDriveStamped, drive_topic, self.drive_callback, 10)
        self.prev_obs_scan = np.full((1080, ), np.inf)
        self.curr_obs_scan = np.full((1080, ), np.inf)
        self.ttc_arr = [np.inf for _ in range(1080)]

        self.curr_state = 'normal'
        self.overtake_time = 0.0
        self.speed = 0.0
        self.steering_ang = 0.0

        self.braking_time = 0.0

    # def obs_scan_callback(self, obs_scan_msg):
    #     self.curr_obs_scan = obs_scan_msg.ranges
    #     self.ttc_arr = [self.curr_obs_scan[i] - self.prev_obs_scan[i] if self.curr_obs_scan[i] != np.inf and self.prev_obs_scan[i] != np.inf else np.inf for i in range(1080)]
    #     self.ttc_arr = [np.abs(self.ttc_arr[i]) if self.ttc_arr[i] < 0 else 0 for i in range(1080)]
    #     self.max_dist_diff = max(self.ttc_arr)
    #     self.prev_obs_scan = self.curr_obs_scan
    #     # print('max dist diff: ', self.max_dist_diff)

    #     # braking transition
    #     if min(self.curr_obs_scan) < self.get_parameter('brake_dist_threshold').get_parameter_value().double_value and self.curr_state == 'normal':
    #         self.curr_state = 'braking'
    #         self.braking_time = time.time()

    #         state_msg = String()
    #         state_msg.data = self.curr_state
    #         self.state_publisher_.publish(state_msg)
    #         return

    #     # braking transition back
    #     if time.time() - self.braking_time >= 3.0 and self.curr_state == 'braking':
    #         self.curr_state = 'normal'

    #         state_msg = String()
    #         state_msg.data = self.curr_state
    #         self.state_publisher_.publish(state_msg)
    #         return

    #     # overtake transition
    #     # if self.max_dist_diff > self.get_parameter('overtake_trigger_dist_diff').get_parameter_value().double_value\
    #     #     and self.steering_ang < self.get_parameter('overtake_trigger_steering_ang').get_parameter_value().double_value  and self.curr_state != 'overtake': 
    #     if self.max_dist_diff > self.get_parameter('overtake_trigger_dist_diff').get_parameter_value().double_value and self.curr_state != 'overtake': 
    #         self.curr_state = 'overtake'
    #         self.overtake_time = time.time()

    #         state_msg = String()
    #         state_msg.data = self.curr_state
    #         self.state_publisher_.publish(state_msg)
    #         return

    #     # overtake transition back
    #     if time.time() - self.overtake_time >= 8.0 and self.curr_state == 'overtake':
    #         self.curr_state = 'normal'

    #         state_msg = String()
    #         state_msg.data = self.curr_state
    #         self.state_publisher_.publish(state_msg)
    #         return

    
    def drive_callback(self, drive_msg):
        self.speed = drive_msg.drive.speed
        self.steering_ang = drive_msg.drive.steering_angle

    def timer_callback(self):
        self.curr_state = 'blocking'
        
        state_msg = String()
        state_msg.data = self.curr_state
        self.state_publisher_.publish(state_msg)

    

def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()