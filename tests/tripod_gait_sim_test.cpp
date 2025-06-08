#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>
#include "test_stubs.h"
#include "../include/locomotion_system.h"

static void printHeader() {
    std::cout << "Idx |";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << " L" << leg + 1 << "C  L" << leg + 1 << "F  L" << leg + 1 << "T |";
    }
    std::cout << std::endl;
}

static void printAngles(int step, LocomotionSystem &sys) {
    std::cout << std::setw(3) << step << " |";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        std::cout << std::fixed << std::setw(6) << std::setprecision(1) << q.coxa << ' '
                  << std::setw(6) << q.femur << ' ' << std::setw(6) << q.tibia << " |";
    }
    std::cout << std::endl;
}

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 7;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65; p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75; p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45; p.tibia_angle_limits[1] = 45;

    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;
    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));

    float velocity = 400.0f; // mm/s
    assert(sys.walkForward(velocity));
    float distance = 800.0f; // mm
    unsigned steps = static_cast<unsigned>((distance / velocity) * p.control_frequency);

    JointAngles prev[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev[i] = sys.getJointAngles(i);
    }

    bool changed = false;
    printHeader();
    for (unsigned s = 0; s < steps; ++s) {
        float phase = static_cast<float>(s) / steps;
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            Point3D pos = sys.calculateFootTrajectory(leg, phase);
            sys.setLegPosition(leg, pos);
        }
        printAngles(s, sys);
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles q = sys.getJointAngles(i);
            if (!changed && (q.coxa != prev[i].coxa || q.femur != prev[i].femur || q.tibia != prev[i].tibia)) {
                changed = true;
            }
            prev[i] = q;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (!changed)
        std::cout << "Warning: servo angles did not change" << std::endl;
    std::cout << "tripod_gait_sim_test executed successfully" << std::endl;
    return 0;
}
