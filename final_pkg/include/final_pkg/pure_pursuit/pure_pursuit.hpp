#pragma once

#include <memory> // for std::unique_ptr
#include <string>
#include <string>
#include "utils/csv_loader.hpp"

class purePursuitHandler{
public:
    purePursuitHandler(std::string waypoints_path);
    virtual ~purePursuitHandler();
    void update_params(double look_ahead_dist, double kp, double speed);
private:
    double look_ahead_dist;
    double kp;
    double speed;
    std::unique_ptr<wayPointLoader> dataloader;
    std::vector<wayPoint> way_points;
};