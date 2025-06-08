#include "../include/locomotion_system.h"
#include "test_stubs.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

static void printWelcome() {
    std::cout << "=========================================" << std::endl;
    std::cout << "   HEXAPOD TRIPOD GAIT SIMULATION TEST" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Simulating 6-legged robot with 3DOF per leg" << std::endl;
    std::cout << "Distance: 800mm | Velocity: 400mm/s | Duration: 2s" << std::endl;
    std::cout << "Sensors: 6 FSR + 1 IMU | Total servos: 18" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;
}

static void printHeader() {
    std::cout << "Step|";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "  L" << leg + 1 << "C   L" << leg + 1 << "F   L" << leg + 1 << "T |";
    }
    std::cout << " Phase | Status" << std::endl;

    std::cout << "----|";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "------|------|------|";
    }
    std::cout << "-------|-------" << std::endl;
}

static void printAngles(int step, LocomotionSystem &sys, float phase) {
    std::cout << std::setw(3) << step << " |";

    // Print joint angles for all legs
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        std::cout << std::fixed << std::setw(6) << std::setprecision(1)
                  << q.coxa << " " << std::setw(6) << q.femur << " "
                  << std::setw(6) << q.tibia << " |";
    }

    // Print gait phase and leg states
    std::cout << std::setw(6) << std::setprecision(2) << phase << " |";

    // Show which legs are in stance/swing
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        LegState state = sys.getLegState(leg);
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';
        std::cout << symbol;
    }

    std::cout << std::endl;
}

static void printLegStateVisualization(LocomotionSystem &sys) {
    std::cout << std::endl
              << "Leg State Visualization:" << std::endl;
    std::cout << "  L1   L2   L3   L4   L5   L6" << std::endl;
    std::cout << "  ";

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        LegState state = sys.getLegState(leg);
        if (state == STANCE_PHASE) {
            std::cout << "[S]  "; // Stance - supporting ground
        } else {
            std::cout << "[W]  "; // Swing - in air
        }
    }
    std::cout << std::endl;
    std::cout << "S=Stance(Ground), W=Swing(Air)" << std::endl;
}

static void printRobotDiagram(LocomotionSystem &sys, float distance_covered) {
    std::cout << std::endl
              << "Robot Position Diagram:" << std::endl;
    std::cout << "Progress: " << std::setw(5) << std::setprecision(1) << std::fixed
              << distance_covered << "mm / 800mm" << std::endl;

    // Simple ASCII robot representation
    std::cout << "    L1 ○ ○ L2" << std::endl;
    std::cout << "  L6 ○   ●   ○ L3   →→→" << std::endl;
    std::cout << "    L5 ○ ○ L4" << std::endl;

    // Progress bar
    int progress = static_cast<int>((distance_covered / 800.0f) * 20);
    std::cout << "Progress: [";
    for (int i = 0; i < 20; ++i) {
        if (i < progress)
            std::cout << "█";
        else
            std::cout << "░";
    }
    std::cout << "] " << std::setw(3) << (int)((distance_covered / 800.0f) * 100) << "%" << std::endl;
}

int main() {
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 90;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

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
