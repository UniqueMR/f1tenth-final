#include <opencv2/opencv.hpp>
#include <vector>

int main() {
    // Load the image
    cv::Mat image = cv::imread("/home/nvidia/f1tenth_ws/src/lab7_pkg/imgs/current_frame.jpg");
    cv::Mat hsv, mask, mask_dilated;

    // Convert to HSV color space
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    // Define the yellow range
    cv::Scalar lower_yellow(18, 60, 110);
    cv::Scalar upper_yellow(30, 280, 280);
    cv::inRange(hsv, lower_yellow, upper_yellow, mask);

    // Highlighting Lane Lines
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask, mask_dilated, kernel, cv::Point(-1, -1), 1);

    // Detecting contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_dilated, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw an outline that matches the characteristics of a rectangle
    cv::Mat lane_image = image.clone();
    for (size_t i = 0; i < contours.size(); i++) {
        double peri = cv::arcLength(contours[i], true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx, 0.02 * peri, true);

        // Getting the aspect ratio
        cv::Rect bounding_rect = cv::boundingRect(approx);
        float aspect_ratio = (float)bounding_rect.width / bounding_rect.height;

        if (aspect_ratio > 1.5) {
            cv::drawContours(lane_image, contours, i, cv::Scalar(0, 255, 0), 3);
        }
    }

    // Show the result
    cv::imshow("Lane Detection", lane_image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    cv::imwrite("lane_detected.png", lane_image);

    return 0;
}

