#include "../include/HexaModel.h"
#include <cassert>
#include <iostream>

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 90;
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    RobotModel model(p);
    auto range = model.calculateHeightRange();
    std::cout << "Min height=" << range.first << " Max height=" << range.second << std::endl;
    assert(range.first <= range.second);
    std::cout << "height_range_test executed successfully" << std::endl;
    return 0;
}
