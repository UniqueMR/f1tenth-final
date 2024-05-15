import cv2
import numpy as np

def main():
    # Load the image
    image = cv2.imread("resource/lane.png")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the yellow range
    lower_yellow = np.array([18, 60, 110])
    upper_yellow = np.array([30, 280, 280])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Highlighting Lane Lines
    kernel = np.ones((5, 5), np.uint8)
    mask_dilated = cv2.dilate(mask, kernel, iterations=1)

    # Detecting contours
    contours, _ = cv2.findContours(mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw an outline that matches the characteristics of a rectangle
    lane_image = image.copy()
    for contour in contours:
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

        # Getting the aspect ratio
        bounding_rect = cv2.boundingRect(approx)
        aspect_ratio = bounding_rect[2] / bounding_rect[3]

        if aspect_ratio > 1.5:
            cv2.drawContours(lane_image, [contour], -1, (0, 255, 0), 3)

    # Show the result
    cv2.imshow("Lane Detection", lane_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save the result
    cv2.imwrite("./imgs/lane_detected.png", lane_image)

if __name__ == "__main__":

    main()
