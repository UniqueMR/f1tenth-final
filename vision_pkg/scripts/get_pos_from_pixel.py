#!/usr/bin/env python
import numpy as np
import cv2
from numpy.linalg import pinv

from intrinsicMat import intrinsicMat 
# from f110_yolo_hw_final import run_inference

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

print("imported")

class GetPosFromPix(Node):

    def __init__(self):
        super().__init__('get_pos_from_pixel')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.on_timer)

        #Get camera matrix
        self.cam_mat  = intrinsicMat()

        self.from_frame_rel = 'ego_racecar/base_link'
        self.to_frame_rel = 'ego_racecar/camera_model'

        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            print("Failed to open camera")
            exit() 
        print('in init')

    def on_timer(self):

        ret, frame = cap.read()
        
        if not ret:
            print("Failed to grab frame")
        
        cv2.imwrite("./imgs/current_frame.jpg", frame)
        cv2.imshow('RealSense', frame)

        #Get center of bounding box
        x, y, w, h, conf = run_inference()

        #Get transformation matrix from tf2
        try:
            t = self.tf_buffer.lookup_transform(
                self.to_frame_rel,
                self.from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.to_frame_rel} to {self.from_frame_rel}: {ex}')
            return

        
        #Do matrix mul to get coordinates in world frame
        quat= t.transform.rotation
        trans= t.transform.translation

        transformation_matrix= np.ones((3, 4))
        transformation_matrix[:3, :3] = np.array([
            [1 - 2 * (quat.z**2 + quat.w**2), 2 * (quat.y * quat.z - quat.x * quat.w), 2 * (quat.y * quat.w + quat.x * quat.z)],
            [2 * (quat.y * quat.z + quat.x * quat.w), 1 - 2 * (quat.y**2 + quat.w**2), 2 * (quat.z * quat.w - quat.x * quat.y)],
            [2 * (quat.y * quat.w - quat.x * quat.z), 2 * (quat.z * quat.w + quat.x * quat.y), 1 - 2 * (quat.y**2 + quat.z**2)]
        ])
        transformation_matrix[:3, 3] = [trans.x, trans.y, trans.z]
        print(transformation_matrix)

        center_car= np.array([x, y, 1]).T
        print((self.cam_mat @ transformation_matrix).shape, center_car.shape)
        coord_in_world = center_car @ (self.cam_mat @ transformation_matrix)
        coord_in_world = coord_in_world/coord_in_world[3]

        print(coord_in_world)
        
def main():

    print('in main')
    rclpy.init()
    get_pos_from_pixel = GetPosFromPix()
    try:
        rclpy.spin(get_pos_from_pixel)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()