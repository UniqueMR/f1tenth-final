#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cmath>

#define PI 3.1415926

class blockingHandler{
public:
    blockingHandler();
    ~blockingHandler();

    void update_params(double kp, double ki, double kd);
    void track_opponent(geometry_msgs::msg::PointStamped::ConstSharedPtr oppo_position_msg);
    void get_pid_control(ackermann_msgs::msg::AckermannDriveStamped &control_msg);
    void pseudo_error_oscillator();

private:
    double kp, ki, kd, t;
    double accum_error, prev_error, curr_error;
    double oppo_position[2];
};