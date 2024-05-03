#include "opponent.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Opponent>());
    rclcpp::shutdown();
    return 0;
}