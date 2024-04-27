import numpy as np
import cv2
import os
import pdb

# Termination criteria for the iterative algorithm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


# Prepare object points based on the chessboard dimensions
checkerboard_size = (6, 8)
objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2) * 25
# pdb.set_trace() 

objpoints = []  # 3d points in real world space
imgpoints = []  # 2d points in image plane

# Load the images
calibration_dir = './calibration'
images = [os.path.join(calibration_dir, fname) for fname in os.listdir(calibration_dir) if fname.endswith('.png')]
# pdb.set_trace()

# Loop over images to find the chessboard corners
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
    # pdb.set_trace()

    # If found, add object points, image points
    if ret:
        objpoints.append(objp)

        # Refine the corner positions
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # pdb.set_trace()
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, checkerboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Perform camera calibration to get camera matrix and distortion coefficients
ret, camera_matrix, distortion_coefficients, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
pdb.set_trace()

# Display the camera calibration parameters
print("Camera matrix:\n", camera_matrix)
print("\nDistortion coefficients:\n", distortion_coefficients)
print("\nRotation Vectors: \n", np.array(rvecs).reshape(14, 3))
print("\nTransition Vectors: \n", np.array(tvecs).reshape(14, 3))