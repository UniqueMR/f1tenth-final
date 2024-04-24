#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import os
import atexit
import tf2_ros
from time import gmtime, strftime
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from numpy import linalg as LA
from nav_msgs.msg import Odometry

# home = expanduser('~')
class WaypointLogger(Node): 

    def __init__(self):
        super().__init__('waypoint_logger')
        self.file = open(strftime('src/pure_pursuit/path/%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
        self.subscription = self.create_subscription(Odometry, '/pf/pose/odom', self.save_waypoint, 10)

    def save_waypoint(self, msg):
        quaternion = np.array([msg.pose.pose.orientation.x, 
                               msg.pose.pose.orientation.y, 
                               msg.pose.pose.orientation.z, 
                               msg.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([msg.twist.twist.linear.x, 
                                  msg.twist.twist.linear.y, 
                                  msg.twist.twist.linear.z]),2)

        # if velocity is positive i.e. car is moving forward
        if msg.twist.twist.linear.x>0.:
            print(msg.twist.twist.linear.x)

        self.file.write('%f, %f, %f, %f\n' % (msg.pose.pose.position.x, # x coor
                                              msg.pose.pose.position.y, # y coor
                                              euler[2],                 # yaw angle
                                              speed))                   # speed

def main(args=None):
    rclpy.init(args=args)

    waypoint_logger = WaypointLogger()

    rclpy.spin(waypoint_logger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
