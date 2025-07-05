#include <iostream>
#include "../src/math_utils.h"
#include "robot_model.h"

int main() {
    std::cout << "=== Test isPointReachable Function ===" << std::endl;

    // Test simple spherical reachability check
    Point3D close_point(100, 100, 100);
    Point3D far_point(1000, 1000, 1000);

    double max_reach = 300.0f;

    bool close_reachable = math_utils::isPointReachable(close_point, max_reach);
    bool far_reachable = math_utils::isPointReachable(far_point, max_reach);

    std::cout << "Point (100,100,100) with max_reach=300mm: " << (close_reachable ? "REACHABLE" : "NOT REACHABLE") << std::endl;
    std::cout << "Point (1000,1000,1000) with max_reach=300mm: " << (far_reachable ? "REACHABLE" : "NOT REACHABLE") << std::endl;

    // Calculate actual distances
    double close_dist = math_utils::magnitude(close_point);
    double far_dist = math_utils::magnitude(far_point);

    std::cout << "\nActual distances:" << std::endl;
    std::cout << "Close point distance: " << close_dist << "mm" << std::endl;
    std::cout << "Far point distance: " << far_dist << "mm" << std::endl;
    std::cout << "Max reach: " << max_reach << "mm" << std::endl;

    return 0;
}
