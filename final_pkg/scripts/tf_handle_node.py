#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import Buffer
from tf2_ros import TransformListener
from nav_msgs.msg import Odometry
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
from builtin_interfaces.msg import Time

class tfHandle(Node):
    def __init__(self):
        super().__init__('tf_handle')

        self.declare_parameter('online', False)
        online = self.get_parameter('online').get_parameter_value().bool_value

        odom_topic = 'pf/pose/odom' if online else 'ego_racecar/odom'
        w2l_transform_topic = 'tf_w2l'
        l2w_transform_topic = 'tf_l2w'
        
        self.parent_frame_id = 'map'
        self.child_frame_id = (online) if online else 'ego_racecar/base_link'

        self.odom_subscriber_ = self.create_subscription(Odometry, odom_topic, self.pose_callback, 10)
        self.tf_w2l_publisher_ = self.create_publisher(TransformStamped, w2l_transform_topic, 10)
        self.tf_l2w_publisher_ = self.create_publisher(TransformStamped, l2w_transform_topic, 10)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.curr_pos_world = PointStamped()
        self.curr_pos_local = PointStamped()
        
    def pose_callback(self, pose_msg):
        try:
            w2l_t = self.tf_buffer_.lookup_transform(self.child_frame_id, self.parent_frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print("Could not transform {} to {}: {}".format(self.parent_frame_id, self.child_frame_id, str(ex)))
            return None

        try:
            l2w_t = self.tf_buffer_.lookup_transform(self.parent_frame_id, self.child_frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print("Could not transform {} to {}: {}".format(self.parent_frame_id, self.child_frame_id, str(ex)))
            return None

        self.curr_pos_world.point.x, self.curr_pos_world.point.y = pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y
        self.curr_pos_local = do_transform_point(self.curr_pos_world, w2l_t)
        print('curr pos local: ', (self.curr_pos_local.point.x, self.curr_pos_local.point.y))

        self.tf_w2l_publisher_.publish(w2l_t)
        self.tf_l2w_publisher_.publish(l2w_t)

def main(args=None):
    rclpy.init(args=args)
    tf_handle = tfHandle()
    rclpy.spin(tf_handle)
    tf_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
