import cv2
import time

cap = cv2.VideoCapture("v4l2src device=/dev/video4 "
                       "extra-controls=\"c,exposure_auto=3\" ! video/x-raw, width=960, height=540, framerate=60/1 ! "
                       "videoconvert ! video/x-raw,format=BGR ! appsink")

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
    
    cv2.imshow('RealSense', frame)
    
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
