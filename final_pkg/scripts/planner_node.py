#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
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
        self.declare_parameter('overtake_trigger_ittc', 3.0)
        self.declare_parameter('overtake_trigger_steering_ang', 1.5)
        self.declare_parameter('brake_dist_threshold', 0.5)
        self.declare_parameter('online', False)
        self.declare_parameter('reward_cnt_reset', 200)
        self.online = self.get_parameter('online').get_parameter_value().bool_value
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        obs_scan_topic = 'obs_scan'
        drive_topic = 'drive'
        odom_topic = 'pf/pose/odom' if self.online else 'ego_racecar/odom'
        reward_topic = 'reward'
        self.obs_scan_subscriber_ = self.create_subscription(LaserScan, obs_scan_topic, self.obs_scan_callback, 10)
        self.drive_subscriber_ = self.create_subscription(AckermannDriveStamped, drive_topic, self.drive_callback, 10)
        self.odom_subscriber_ = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.reward_publisher_ = self.create_publisher(Float64, reward_topic, 10)

        self.prev_obs_scan = np.full((1080, ), np.inf)
        self.curr_obs_scan = np.full((1080, ), np.inf)
        self.dist_diff_arr = [np.inf for _ in range(1080)]
        self.ittc_arr = [np.inf for _ in range(1080)]
        self.min_ittc = np.inf

        self.curr_state = 'normal'
        self.overtake_time = 0.0
        self.speed = 0.0
        self.steering_ang = 0.0

        self.braking_time = 0.0
        centerline = pd.read_csv('./src/final_pkg/path/centerline.csv', sep=',', usecols=[0,1,3])
        self.traj_x, self.traj_y, self.traj_theta, self.traj_s = np.array(centerline.iloc[:, 0]), np.array(centerline.iloc[:, 1]), np.array(centerline.iloc[:, 2]), []
        self.get_traj_s()
        self.reward_stamp, self.reward_accum = 0.0, 0.0
        self.prev_s = 0.0
        self.prev_idx_next = 0
        self.reward_cnt = 0

    def obs_scan_callback(self, obs_scan_msg):
        self.curr_obs_scan = obs_scan_msg.ranges
        self.dist_diff_arr = [self.curr_obs_scan[i] - self.prev_obs_scan[i] if self.curr_obs_scan[i] != np.inf and self.prev_obs_scan[i] != np.inf else np.inf for i in range(1080)]
        self.dist_diff_arr = [np.abs(self.dist_diff_arr[i]) if self.dist_diff_arr[i] < 0 else 0 for i in range(1080)]
        self.ittc_arr = [self.curr_obs_scan[i] / self.dist_diff_arr[i] if self.dist_diff_arr[i] != 0 else np.inf for i in range(1080)]
        
        self.prev_obs_scan = self.curr_obs_scan
        # print('max dist diff: ', self.max_dist_diff)

        print(min(self.ittc_arr))

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
        if min(self.ittc_arr) < self.get_parameter('overtake_trigger_ittc').get_parameter_value().double_value and self.curr_state != 'overtake': 
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
        s, _, idx_last, idx_next = self.cartesian_to_frenet(pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)

        self.reward_stamp = s - self.prev_s if s > self.prev_s else self.traj_s[-1] - self.prev_s + s
        self.reward_accum += self.reward_stamp
        self.reward_cnt += 1

        if self.reward_cnt >= self.get_parameter('reward_cnt_reset').get_parameter_value().integer_value:
            reward_msg = Float64()
            reward_msg.data = self.reward_accum
            self.reward_publisher_.publish(reward_msg)
            self.reward_accum = 0.0
            self.reward_cnt = 0
        
        self.prev_s = s
        self.prev_idx_next = idx_next
        return

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
        idx_last, idx_next = self.find_closest_segment(x, y)
        # Project the point onto the segment for s and d
        segment_vector = np.array([self.traj_x[idx_next] - self.traj_x[idx_last], self.traj_y[idx_next] - self.traj_y[idx_last]])
        point_vector = np.array([x - self.traj_x[idx_last], y - self.traj_y[idx_last]])
        proj_length = np.dot(point_vector, segment_vector) / np.linalg.norm(segment_vector)
        # Use cross product to consider the direction of the point relative to the segment
        # numba use cross2d instead of np.cross()
        d = np.cross(segment_vector, point_vector) / np.linalg.norm(segment_vector)
        # d = cross2d(segment_vector, point_vector) / np.linalg.norm(segment_vector)
        s = self.traj_s[idx_last] + proj_length
        return s, d, idx_last, idx_next

    def find_closest_segment(self, x, y):
        """
        Find the closest segment on a trajectory to a given point.
        Assume pass in trajectory has overlapped point(start & end)
        """
        # Cut the overlapped point at the end to avoid edge case
        n = len(self.traj_x) - 1
        distances = np.sqrt((self.traj_x[:-1] - x) ** 2 + (self.traj_y[:-1] - y) ** 2)
        idx_closest = np.argmin(distances)
        head_vector = np.array([np.cos(self.traj_theta[idx_closest]), np.sin(self.traj_theta[idx_closest])])
        point_vector = np.array([x - self.traj_x[idx_closest], y - self.traj_y[idx_closest]])
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