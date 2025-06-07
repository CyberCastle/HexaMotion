#include <cassert>
#include <iostream>
#include "../include/model.h"

int main() {
    Parameters p{};
    RobotModel model_bad(p);
    assert(!model_bad.validate());

    p.hexagon_radius = 100;
    p.coxa_length = 30;
    p.femur_length = 50;
    p.tibia_length = 70;
    p.robot_height = 100;
    p.control_frequency = 50;
    RobotModel model_ok(p);
    assert(model_ok.validate());
    std::cout << "model_test executed successfully" << std::endl;
    return 0;
}
