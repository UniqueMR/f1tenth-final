#include "blocking/blocking.hpp"

blockingHandler::blockingHandler(){
    accum_error = 0.0, prev_error = 0.0, curr_error = 0.0, t = 0.0;
}

void blockingHandler::update_params(double kp, double ki, double kd){
    this->kp = kp, this->ki = ki, this->kd = kd;
}

void blockingHandler::track_opponent(geometry_msgs::msg::PointStamped::ConstSharedPtr oppo_position_msg){
    oppo_position[0] = oppo_position_msg->point.x;
    oppo_position[1] = oppo_position_msg->point.y;
}

void blockingHandler::get_pid_control(ackermann_msgs::msg::AckermannDriveStamped &control_msg){
    double p = kp * curr_error;
    double i = ki * accum_error;
    double d = kd * (curr_error - prev_error);
    prev_error = curr_error;
    accum_error += curr_error;
    double angle = -(p + i + d);

    // Determine speed according to steering angle
    double abs_angle = std::fabs(angle);
    double speed = 0.0;
    if(abs_angle < 10.0 * PI / 180.0) speed = 4.5;
    else if(abs_angle < 20.0 * PI / 180.0)  speed = 3.5;
    else speed = 2.5;

    // Publish drive msg to a topic
    // TODO: fill in drive message and publish
    control_msg.drive.speed = speed;
    control_msg.drive.steering_angle = angle;
}

void blockingHandler::pseudo_error_oscillator(){
    double amplitude = 0.25;
    double frequency = 0.01;
    double dt = 1.0;

    t += dt;
    curr_error = amplitude * sin(2 * M_PI * frequency * t);
}

blockingHandler::~blockingHandler(){

}
