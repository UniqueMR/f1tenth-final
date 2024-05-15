#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0); // Open the default camera
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cv::Mat frame;
    std::cout << "Start grabbing" << std::endl
              << "Press ESC to terminate" << std::endl;

    while (true) {
        cap.read(frame); // Capture a new frame
        if (frame.empty()) {
            std::cerr << "ERROR! blank frame grabbed\n";
            break;
        }
        cv::imshow("Live", frame); // Show the frame in a window
        if (cv::waitKey(5) == 27) break; // Wait for 'ESC' key press for 5ms. If 'ESC' key is pressed, break loop
    }

    // When everything done, release the capture
    cap.release();
    cv::destroyAllWindows();
    return 0;
}

