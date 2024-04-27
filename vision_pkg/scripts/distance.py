import numpy as np
import cv2
import pdb


def calculate_distances(known_point_image, known_distance, unknown_point_image):

    # Given camera matrix and distortion coefficients
    camera_matrix = np.array([[694.71557753, 0., 449.37516845],
                            [0., 695.54966923, 258.6472499],
                            [0., 0., 1.]])
    distortion_coefficients = np.array([0.14755283, 0.1924631, -0.00918369, -0.01212976, -1.706382141])
    
    known_point_image = known_point_image.reshape(1, -1, 2)
    unknown_point_image = unknown_point_image.reshape(1, -1, 2)

    # pdb.set_trace()

    known_point_undistorted = cv2.undistortPoints(known_point_image, camera_matrix, distortion_coefficients)
    unknown_point_undistorted = cv2.undistortPoints(unknown_point_image, camera_matrix, distortion_coefficients)

    x_known, y_known = known_point_undistorted[0][0]

    # angle_known = np.arctan2(x_known, camera_matrix[0, 0])
    # the x_known is already in normalized coordinate
    # I think it doesn't need to divide fx again
    angle_known = np.arctan2(x_known, 1) 

    camera_height = known_distance * np.tan(angle_known)

    x_unknown, y_unknown = unknown_point_undistorted[0][0]
    angle_unknown = np.arctan2(x_unknown, 1)
    distance_unknown = camera_height / np.tan(angle_unknown)

    return camera_height, distance_unknown
    

known_distance = 40  # distance in cm
known_point_image = np.array([[666, 495]], dtype=np.float32)
unknown_point_image = np.array([[179.6079978942871, 100.0]], dtype=np.float32)

camera_height, distance_to_unknown_cone = calculate_distances(known_point_image, known_distance, unknown_point_image)

print(f"Camera mounting height: {camera_height} cm")
print(f"Distance to the unknown cone: {distance_to_unknown_cone} cm")
