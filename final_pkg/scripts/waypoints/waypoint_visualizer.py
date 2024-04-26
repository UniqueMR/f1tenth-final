#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv

class WaypointVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_visualizer')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.01, self.publish_markers)
        self.marker_id = 0

    def publish_markers(self):
        # Read CSV file
        with open('src/final_pkg/path/using.csv', 'r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                if len(row) >= 2:
                    x, y = float(row[1]), float(row[2])
                    self.publish_marker(x, y)

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.publisher.publish(marker)
        self.marker_id += 1

def main(args=None):
    rclpy.init(args=args)
    waypoint_visualizer = WaypointVisualizer()
    rclpy.spin(waypoint_visualizer)
    waypoint_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



