#include "utils/csv_loader.hpp"

wayPoint::wayPoint(double x, double y)
{
    this->x = x;
    this->y = y;
}

wayPointLoader::wayPointLoader(const std::string file_path)
{
    std::ifstream file(file_path);

    if(!file.is_open()) throw std::runtime_error("could not open csv file" + file_path);

    std::string line;
    while (getline(file, line)) 
    {
        this->way_points.push_back(parse_csv_line(line));
    }
}

wayPoint wayPointLoader::parse_csv_line(const std::string line)
{
    std::istringstream s(line);
    std::string field;

    double x, y;

    // Assuming CSV format is: x,y
    getline(s, field, ',');
    x = std::stod(field);
    
    getline(s, field);
    y = std::stod(field);

    return wayPoint(x, y);
}