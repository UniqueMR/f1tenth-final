#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import tf2_ros
import gym

class simMap(Node):
    def __init__(self):
        super().__init__("sim_map")
        self.declare_parameter('sim_map_path', '')
        self.declare_parameter('sim_map_img_ext', '.png')
        self.declare_parameter('sim_num_agents', 1)
        self.declare_parameter('online', False)

        pose_topic = "pf/pose/odom" if self.get_parameter('online').get_parameter_value().bool_value else "ego_racecar/odom"
        sim_scan_topic = 'sim_scan'
        # create publisher and subscriber  
        self.odom_subscriber_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, 10)
        self.sim_scan_publisher_ = self.create_publisher(LaserScan, sim_scan_topic, 10)
        
        # create transformation
        self.parent_frame_id = 'map'
        self.child_frame_id = 'laser' if self.get_parameter('online').get_parameter_value().bool_value else 'ego_racecar/base_link'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # env backend
        self.sx, self.sy, self.stheta = 0, 0, 0
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('sim_map_path').value,
                            map_ext=self.get_parameter('sim_map_img_ext').value,
                            num_agents=self.get_parameter('sim_num_agents').get_parameter_value().integer_value)

        self.obs, _, self.done, _ = self.env.reset(np.array([[self.sx, self.sy, self.stheta]]))

    def pose_callback(self, pose_msg):
        # update transform
        try:
            transform = self.tf_buffer.lookup_transform(self.parent_frame_id, self.child_frame_id, rclpy.time.Time())
            # update the vehicle's current state 
            self.sx = pose_msg.pose.pose.position.x
            self.sy = pose_msg.pose.pose.position.y
            self.stheta = self.get_yaw(transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn('Failed to lookup transform')

        self.obs, _, self.done, _ = self.env.reset(np.array([[self.sx, self.sy, self.stheta]]))

        scan_msg = LaserScan()
        scan_msg.header.frame_id = self.child_frame_id
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.angle_min = -2.3499999046325684
        scan_msg.angle_max = 2.3499999046325684
        scan_msg.angle_increment = 0.004351851996034384
        scan_msg.range_min = 0.0
        scan_msg.range_max = 30.0
        scan_msg.ranges = self.obs['scans'][0].tolist()
        self.sim_scan_publisher_.publish(scan_msg)

    def get_yaw(self, t):
        q = t.transform.rotation
        _, _, yaw = self.quaternion_to_euler(q)
        return yaw

    def quaternion_to_euler(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w        
        # Perform calculations
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

def main(args=None):
    rclpy.init(args=args)
    sim_map = simMap()
    rclpy.spin(sim_map)
    sim_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()