#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Planner(Node):
    def __init__(self):
        super().__init__('planner')
        self.state_publisher_ = self.create_publisher(String, 'strategy', 10)
        self.declare_parameter("timer_period", 20.0)
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.curr_state = 'normal'

    def timer_callback(self):
        # this is just a dummy planner
        # set it to maintain a certain state for single functionality test
        # if self.curr_state == 'normal':
        #     self.curr_state = 'overtake'
        # elif self.curr_state == 'overtake':
        #     self.curr_state = 'normal'
        # else:
        #     pass
        self.curr_state = 'overtake'
        
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