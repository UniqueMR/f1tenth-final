#include "rrt/rrt.hpp"

void compareOccupancyGridData(const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid1,
                              const std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid2) {
    
    // Check if the sizes of the data vectors match
    if (grid1->data.size() != grid2->data.size()) {
        std::cerr << "Data size mismatch: "
                  << "grid1 size = " << grid1->data.size() << ", "
                  << "grid2 size = " << grid2->data.size() << std::endl;
        return;
    }

    // Compare the data vectors
    bool differences_found = false;
    for (size_t i = 0; i < grid1->data.size(); ++i) {
        if (grid1->data[i] != grid2->data[i]) {
            differences_found = true;
            std::cout << "Difference at (" << i / 759 << ", " << i % 759 << "): "
                      << "grid1->data = " << static_cast<int>(grid1->data[i]) << ", "
                      << "grid2->data = " << static_cast<int>(grid2->data[i]) << std::endl;
        }
    }

    if (!differences_found) {
        std::cout << "No differences found in the data vectors." << std::endl;
    }

}