#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import time
from numba.np.extensions import cross2d
import pandas as pd
import pdb

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.state_publisher_ = self.create_publisher(String, 'strategy', 10)
        self.declare_parameter("timer_period", 20.0)
        self.declare_parameter('overtake_trigger_dist_diff', 3.0)
        self.declare_parameter('overtake_trigger_steering_ang', 1.5)
        self.declare_parameter('brake_dist_threshold', 0.5)
        self.declare_parameter('online', False)
        self.online = self.get_parameter('online').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        obs_scan_topic = 'obs_scan'
        drive_topic = 'drive'
        odom_topic = 'pf/pose/odom' if self.online else 'ego_racecar/odom'
        self.obs_scan_subscriber_ = self.create_subscription(LaserScan, obs_scan_topic, self.obs_scan_callback, 10)
        self.drive_subscriber_ = self.create_subscription(AckermannDriveStamped, drive_topic, self.drive_callback, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.prev_obs_scan = np.full((1080, ), np.inf)
        self.curr_obs_scan = np.full((1080, ), np.inf)
        self.ttc_arr = [np.inf for _ in range(1080)]

        self.curr_state = 'normal'
        self.overtake_time = 0.0
        self.speed = 0.0
        self.steering_ang = 0.0

        self.braking_time = 0.0
        centerline = pd.read_csv('./src/final_pkg/path/centerline.csv', sep=',', usecols=[0,1,3])
        self.traj_x, self.traj_y, self.traj_theta, self.traj_s = np.array(centerline.iloc[:, 0]), np.array(centerline.iloc[:, 1]), np.array(centerline.iloc[:, 2]), []
        self.get_traj_s()

    def obs_scan_callback(self, obs_scan_msg):
        self.curr_obs_scan = obs_scan_msg.ranges
        self.ttc_arr = [self.curr_obs_scan[i] - self.prev_obs_scan[i] if self.curr_obs_scan[i] != np.inf and self.prev_obs_scan[i] != np.inf else np.inf for i in range(1080)]
        self.ttc_arr = [np.abs(self.ttc_arr[i]) if self.ttc_arr[i] < 0 else 0 for i in range(1080)]
        self.max_dist_diff = max(self.ttc_arr)
        self.prev_obs_scan = self.curr_obs_scan
        # print('max dist diff: ', self.max_dist_diff)

        # braking transition
        if min(self.curr_obs_scan) < self.get_parameter('brake_dist_threshold').get_parameter_value().double_value and self.curr_state == 'normal':
            self.curr_state = 'braking'
            self.braking_time = time.time()

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)
            return

        # braking transition back
        if time.time() - self.braking_time >= 3.0 and self.curr_state == 'braking':
            self.curr_state = 'normal'

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)
            return

        # overtake transition
        # if self.max_dist_diff > self.get_parameter('overtake_trigger_dist_diff').get_parameter_value().double_value\
        #     and self.steering_ang < self.get_parameter('overtake_trigger_steering_ang').get_parameter_value().double_value  and self.curr_state != 'overtake': 
        if self.max_dist_diff > self.get_parameter('overtake_trigger_dist_diff').get_parameter_value().double_value and self.curr_state != 'overtake': 
            self.curr_state = 'overtake'
            self.overtake_time = time.time()

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)
            return

        # overtake transition back
        if time.time() - self.overtake_time >= 8.0 and self.curr_state == 'overtake':
            self.curr_state = 'normal'

            state_msg = String()
            state_msg.data = self.curr_state
            self.state_publisher_.publish(state_msg)
            return

    def odom_callback(self, pose_msg):
        s, d = self.cartesian_to_frenet(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
        print('s, d: ', s, d)
    
    def drive_callback(self, drive_msg):
        self.speed = drive_msg.drive.speed
        self.steering_ang = drive_msg.drive.steering_angle

    def cartesian_to_frenet(self, x, y):
        """
        Args:
            pose: [x, y, ...] in Cartesian frame
            traj_x, traj_y, traj_s: Discretized waypoints in Frenet frame
        Returns:
            s, d
        """
        idx_last, idx_next = self.find_closest_segment(x, y, self.traj_x, self.traj_y, self.traj_theta)
        # Project the point onto the segment for s and d
        segment_vector = np.array([self.traj_x[idx_next] - self.traj_x[idx_last], self.traj_y[idx_next] - self.traj_y[idx_last]])
        point_vector = np.array([x - self.traj_x[idx_last], y - self.traj_y[idx_last]])
        proj_length = np.dot(point_vector, segment_vector) / np.linalg.norm(segment_vector)
        # Use cross product to consider the direction of the point relative to the segment
        # numba use cross2d instead of np.cross()
        d = np.cross(segment_vector, point_vector) / np.linalg.norm(segment_vector)
        # d = cross2d(segment_vector, point_vector) / np.linalg.norm(segment_vector)
        s = self.traj_s[idx_last] + proj_length
        return s, d

    def find_closest_segment(self, x, y, traj_x, traj_y, traj_theta):
        """
        Find the closest segment on a trajectory to a given point.
        Assume pass in trajectory has overlapped point(start & end)
        """
        # Cut the overlapped point at the end to avoid edge case
        n = len(traj_x) - 1
        distances = np.sqrt((traj_x[:-1] - x) ** 2 + (traj_y[:-1] - y) ** 2)
        idx_closest = np.argmin(distances)
        head_vector = np.array([np.cos(traj_theta[idx_closest]), np.sin(traj_theta[idx_closest])])
        point_vector = np.array([x - traj_x[idx_closest], y - traj_y[idx_closest]])
        dot_head_point = np.dot(head_vector, point_vector)
        if dot_head_point >= 0:
            idx_last = idx_closest
            idx_next = idx_closest + 1
        else:
            idx_next = idx_closest
            idx_last = n-1 if idx_closest == 0 else idx_closest - 1

        return idx_last, idx_next
    
    def get_traj_s(self):
        self.traj_s.append(0.0)
        try:
            assert len(self.traj_x) == len(self.traj_y), 'unequal length in trajectory'
        except AssertionError as error:
            print(f'Error: {error}')

        for i in range(1, len(self.traj_x)):
            self.traj_s.append(self.traj_s[i - 1] + np.sqrt((self.traj_x[i] - self.traj_x[i - 1]) ** 2 + (self.traj_y[i] - self.traj_y[i - 1]) ** 2))
        self.traj_s = np.array(self.traj_s)
        return

    # def timer_callback(self):
    #     self.curr_state = 'blocking'
        
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