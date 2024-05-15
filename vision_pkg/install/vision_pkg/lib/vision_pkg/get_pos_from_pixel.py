import numpy as np
import cv2

from intrinsicMat import intrinsicMat 

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class GetPosFromPix():

    def __init__(self):

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        #Get camera matrix
        self.cam_mat, _ , _ , _  = intrinsicMat()

        self.from_frame_rel = 'ego_racecar/base_link'
        self.to_frame_rel = 'ego_racecar/camera_model'

        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            print("Failed to open camera")
            exit() 

    def on_timer(self):

        # ret, frame = cap.read()
        
        # if not ret:
        #     print("Failed to grab frame")
        #     break
        
        # cv2.imwrite("./imgs/current_frame.jpg", frame)
        # cv2.imshow('RealSense', frame)

        #Get center of bounding box
        x, y, w, h, conf = run_inference()

        #Get transformation matrix from tf2
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        rot_mat= t.transform.rotation
        print(rot_mat)
        #Do matrix mul to get coordinates in world frame

def main():
    rclpy.init()
    node = GetPosFromPix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()