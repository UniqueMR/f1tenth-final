#pragma once

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class wayPoint 
{
public:
    double x, y; // Add more attributes as needed

    wayPoint(double x, double y);
};

class wayPointLoader
{
public:
    std::vector<wayPoint> way_points;
    wayPointLoader(const std::string file_path);

private:
    wayPoint parse_csv_line(const std::string line);
};