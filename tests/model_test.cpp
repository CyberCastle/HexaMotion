#include "robot_model.h"
#include <cassert>
#include <iostream>

int main() {
    Parameters p{};
    RobotModel model_bad(p);
    assert(!model_bad.validate());

    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.height_offset = 0;
    p.control_frequency = 50;
    RobotModel model_ok(p);
    assert(model_ok.validate());
    std::cout << "model_test executed successfully" << std::endl;
    return 0;
}
