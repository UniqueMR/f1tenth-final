from distance import calculate_distances
from f110_yolo_hw_final import run_inference
from lane import lane
import cv2
import time
import pdb

from pathlib import Path
from PIL import Image
import numpy as np
from copy import deepcopy
import tensorrt as trt
import torch
import matplotlib as plt
import pycuda.autoinit
import pycuda.driver as cuda
import matplotlib.patches as patches
from typing import Tuple
import time


cap = cv2.VideoCapture(4)

# Check if the camera is successfully opened
if not cap.isOpened():
    print("Failed to open camera")
    exit() 

# Initialise timer and frame counter
time_start = time.perf_counter()
frame_count = 0

while True:
    # Read a frame
    ret, frame = cap.read()
    
    if not ret:
        print("Failed to grab frame")
        break
    
    cv2.imwrite("./imgs/current_frame.jpg", frame)
    cv2.imshow('RealSense', frame)
    
    # lane detection
    lane()

    # get pixel coord of car's center
    x, y, w, h, conf = run_inference()

    if w == 0 and h == 0:
    # calculate distance to car
        known_distance = 40  # distance in cm
        known_point_image = np.array([[666, 495]], dtype=np.float32)
        unknown_point_image = np.array([[x, y]], dtype=np.float32)
        camera_height, distance_to_car = calculate_distances(known_point_image, known_distance, unknown_point_image)

        print("Distance to car: ", distance_to_car)
        
    # Update the frame count
    frame_count += 1
    # Average FPS calculated every 60 frames and printed
    if frame_count >= 60:  
        time_end = time.perf_counter()  
        fps = frame_count / (time_end - time_start)  
        print(f'Average FPS: {fps:.2f}')
        frame_count = 0  #  reset the frame counter
        time_start = time.perf_counter()  # reset timer
    
    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
