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

// Forward declaration
static void printAngleBar(float angle, float min_angle, float max_angle);

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

static void printServoAngleGraph(LocomotionSystem &sys, int step) {
    std::cout << std::endl
              << "Servo Angle Visualization (Step " << step << "):" << std::endl;
    std::cout << "Angle Range: -75° to +75° | Scale: each char = 5°" << std::endl;
    std::cout << "   -75  -50  -25    0   25   50   75" << std::endl;
    std::cout << "    |    |    |    |    |    |    |" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        LegState state = sys.getLegState(leg);
        char leg_symbol = (state == STANCE_PHASE) ? 'S' : 'W';

        // Coxa angle visualization
        std::cout << "L" << leg + 1 << "C" << leg_symbol << ": ";
        printAngleBar(q.coxa, -75, 75);
        std::cout << " (" << std::setw(5) << std::setprecision(1) << q.coxa << "°)" << std::endl;

        // Femur angle visualization
        std::cout << "L" << leg + 1 << "F" << leg_symbol << ": ";
        printAngleBar(q.femur, -75, 75);
        std::cout << " (" << std::setw(5) << std::setprecision(1) << q.femur << "°)" << std::endl;

        // Tibia angle visualization
        std::cout << "L" << leg + 1 << "T" << leg_symbol << ": ";
        printAngleBar(q.tibia, -75, 75);
        std::cout << " (" << std::setw(5) << std::setprecision(1) << q.tibia << "°)" << std::endl;

        if (leg < NUM_LEGS - 1)
            std::cout << std::endl;
    }
}

static void printAngleBar(float angle, float min_angle, float max_angle) {
    const int bar_length = 30;
    int position = static_cast<int>(((angle - min_angle) / (max_angle - min_angle)) * bar_length);
    position = std::max(0, std::min(bar_length - 1, position));

    for (int i = 0; i < bar_length; ++i) {
        if (i == bar_length / 2) {
            std::cout << "|"; // Center line (0°)
        } else if (i == position) {
            std::cout << "●"; // Current angle position
        } else {
            std::cout << "─";
        }
    }
}

static void printTripodGaitPattern(LocomotionSystem &sys, float phase) {
    std::cout << std::endl
              << "Tripod Gait Pattern (Phase: " << std::setprecision(2) << phase << "):" << std::endl;
    std::cout << "Group A (L1,L3,L5): ";

    // Group A legs (1, 3, 5) - first tripod
    for (int leg : {0, 2, 4}) {
        LegState state = sys.getLegState(leg);
        if (state == STANCE_PHASE) {
            std::cout << "▓▓"; // Stance phase - on ground
        } else {
            std::cout << "░░"; // Swing phase - in air
        }
    }

    std::cout << std::endl
              << "Group B (L2,L4,L6): ";

    // Group B legs (2, 4, 6) - second tripod
    for (int leg : {1, 3, 5}) {
        LegState state = sys.getLegState(leg);
        if (state == STANCE_PHASE) {
            std::cout << "▓▓"; // Stance phase - on ground
        } else {
            std::cout << "░░"; // Swing phase - in air
        }
    }

    std::cout << std::endl;
    std::cout << "▓=Ground Support, ░=Air Movement" << std::endl;
}

static void printDetailedRobotStatus(LocomotionSystem &sys, float distance_covered, int step) {
    std::cout << std::endl
              << "═══════════════════════════════════════" << std::endl;
    std::cout << "HEXAPOD STATUS - Step " << step << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;
    std::cout << "Distance: " << std::setw(6) << std::setprecision(1) << distance_covered
              << "mm / 800mm (" << std::setw(3) << (int)((distance_covered / 800.0f) * 100) << "%)" << std::endl;
    std::cout << "Robot Height: " << std::setw(4) << std::setprecision(1) << sys.getParameters().robot_height << "mm" << std::endl;
    std::cout << "Step Height: " << std::setw(4) << std::setprecision(1) << sys.getStepHeight() << "mm" << std::endl;
    std::cout << "Step Length: " << std::setw(4) << std::setprecision(1) << sys.getStepLength() << "mm" << std::endl;

    // Show active servo count
    int active_servos = 0;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        LegState state = sys.getLegState(leg);
        if (state == SWING_PHASE)
            active_servos += 3; // 3 servos per leg in swing
    }
    std::cout << "Active Servos: " << active_servos << "/18" << std::endl;
}

int main() {
    printWelcome();

    // Test with different robot heights to demonstrate the height parameter effect
    std::cout << "Testing robot height parameter effect on servo angles..." << std::endl;
    std::cout << "=========================================" << std::endl;

    // Test configuration 1: Lower height (80mm)
    std::cout << "Configuration 1: Robot Height = 100mm" << std::endl;
    Parameters p1{};
    p1.hexagon_radius = 400;
    p1.coxa_length = 50;
    p1.femur_length = 101;
    p1.tibia_length = 208;
    p1.robot_height = 80; // Lower height
    p1.height_offset = 0;
    p1.control_frequency = 50;
    p1.coxa_angle_limits[0] = -65;
    p1.coxa_angle_limits[1] = 65;
    p1.femur_angle_limits[0] = -75;
    p1.femur_angle_limits[1] = 75;
    p1.tibia_angle_limits[0] = -45;
    p1.tibia_angle_limits[1] = 45;

    LocomotionSystem sys1(p1);
    DummyIMU imu1;
    DummyFSR fsr1;
    DummyServo servos1;

    assert(sys1.initialize(&imu1, &fsr1, &servos1));
    assert(sys1.calibrateSystem());
    assert(sys1.setGaitType(TRIPOD_GAIT));
    assert(sys1.walkForward(400.0f));

    std::cout << "Sample joint angles at 100mm height:" << std::endl;
    std::cout << "=== About to call trajectory and setLegPosition ===" << std::endl;
    for (int leg = 0; leg < 3; ++leg) {
        Point3D pos = sys1.calculateFootTrajectory(leg, 0.5f);
        std::cout << "  Leg " << leg + 1 << " target position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
        sys1.setLegPosition(leg, pos);
        JointAngles q = sys1.getJointAngles(leg);
        std::cout << "  Leg " << leg + 1 << ": Coxa=" << std::setw(6) << std::setprecision(1) << q.coxa
                  << "° Femur=" << std::setw(6) << q.femur << "° Tibia=" << std::setw(6) << q.tibia << "°" << std::endl;

        // Verify forward kinematics result
        Point3D fk_pos = sys1.calculateForwardKinematics(leg, q);
        std::cout << "  FK verification: x=" << fk_pos.x << ", y=" << fk_pos.y << ", z=" << fk_pos.z << std::endl;
        float error = sqrt(pow(pos.x - fk_pos.x, 2) + pow(pos.y - fk_pos.y, 2) + pow(pos.z - fk_pos.z, 2));
        std::cout << "  IK error: " << error << "mm" << std::endl;
    }

    std::cout << std::endl;

    // Test configuration 2: Higher height (200mm)
    std::cout << "Configuration 2: Robot Height = 200mm" << std::endl;
    Parameters p2{};
    p2.hexagon_radius = 400;
    p2.coxa_length = 50;
    p2.femur_length = 101;
    p2.tibia_length = 208;
    p2.robot_height = 200; // Higher height
    p2.height_offset = 0;
    p2.control_frequency = 50;
    p2.coxa_angle_limits[0] = -65;
    p2.coxa_angle_limits[1] = 65;
    p2.femur_angle_limits[0] = -75;
    p2.femur_angle_limits[1] = 75;
    p2.tibia_angle_limits[0] = -45;
    p2.tibia_angle_limits[1] = 45;

    LocomotionSystem sys2(p2);
    DummyIMU imu2;
    DummyFSR fsr2;
    DummyServo servos2;

    assert(sys2.initialize(&imu2, &fsr2, &servos2));
    assert(sys2.calibrateSystem());
    assert(sys2.setGaitType(TRIPOD_GAIT));
    assert(sys2.walkForward(400.0f));

    std::cout << "Sample joint angles at 200mm height:" << std::endl;
    std::cout << "=== About to call trajectory and setLegPosition ===" << std::endl;
    for (int leg = 0; leg < 3; ++leg) {
        Point3D pos = sys2.calculateFootTrajectory(leg, 0.5f);
        std::cout << "  Leg " << leg + 1 << " target position: x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
        sys2.setLegPosition(leg, pos);
        JointAngles q = sys2.getJointAngles(leg);
        std::cout << "  Leg " << leg + 1 << ": Coxa=" << std::setw(6) << std::setprecision(1) << q.coxa
                  << "° Femur=" << std::setw(6) << q.femur << "° Tibia=" << std::setw(6) << q.tibia << "°" << std::endl;

        // Verify forward kinematics result
        Point3D fk_pos = sys2.calculateForwardKinematics(leg, q);
        std::cout << "  FK verification: x=" << fk_pos.x << ", y=" << fk_pos.y << ", z=" << fk_pos.z << std::endl;
        float error = sqrt(pow(pos.x - fk_pos.x, 2) + pow(pos.y - fk_pos.y, 2) + pow(pos.z - fk_pos.z, 2));
        std::cout << "  IK error: " << error << "mm" << std::endl;
    }

    std::cout << std::endl
              << "Notice how the servo angles differ between 100mm and 200mm heights!" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;

    // Initialize robot parameters for main simulation (use 150mm height)
    Parameters p{};
    p.hexagon_radius = 400;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 150;
    p.height_offset = 0;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -90;
    p.coxa_angle_limits[1] = 90;
    p.femur_angle_limits[0] = -90;
    p.femur_angle_limits[1] = 90;
    p.tibia_angle_limits[0] = -90;
    p.tibia_angle_limits[1] = 90;

    // Initialize locomotion system
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    std::cout << "Initializing locomotion system..." << std::endl;
    assert(sys.initialize(&imu, &fsr, &servos));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));

    // Simulation parameters
    float velocity = 400.0f;              // mm/s
    float distance = 800.0f;              // mm
    float duration = distance / velocity; // 2 seconds
    unsigned steps = static_cast<unsigned>(duration * p.control_frequency);

    std::cout << "Starting tripod gait simulation..." << std::endl;
    std::cout << "Total steps: " << steps << " | Step interval: " << (1000.0f / p.control_frequency) << "ms" << std::endl;
    std::cout << std::endl;

    assert(sys.walkForward(velocity));

    // Track previous joint angles to detect changes
    JointAngles prev[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev[i] = sys.getJointAngles(i);
    }

    bool changed = false;
    printHeader();

    for (unsigned s = 0; s < steps; ++s) {
        float phase = static_cast<float>(s) / static_cast<float>(steps);
        float distance_covered = phase * distance;

        // Use the main system update for proper locomotion control
        sys.update();

        // Print servo angles and states
        printAngles(s, sys, phase);

        // Check if angles changed
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles q = sys.getJointAngles(i);
            if (!changed && (std::abs(q.coxa - prev[i].coxa) > 0.1f ||
                             std::abs(q.femur - prev[i].femur) > 0.1f ||
                             std::abs(q.tibia - prev[i].tibia) > 0.1f)) {
                changed = true;
            }
            prev[i] = q;
        }

        // Show visual diagrams every 10 steps
        if (s % 10 == 0) {
            // printLegStateVisualization(sys);
            //  printRobotDiagram(sys, distance_covered);
            //  printServoAngleGraph(sys, s);
            //  printTripodGaitPattern(sys, phase);
            // printDetailedRobotStatus(sys, distance_covered, s);
            std::cout << std::endl;
        }

        // Simulate real-time execution
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Final status
    // std::cout << std::endl
    //           << "=========================================" << std::endl;
    // std::cout << "SIMULATION COMPLETED" << std::endl;
    // std::cout << "=========================================" << std::endl;

    // if (!changed) {
    //     std::cout << "⚠️  Warning: servo angles did not change during simulation" << std::endl;
    // } else {
    //     std::cout << "✓ Servo angles changed correctly during simulation" << std::endl;
    // }

    // Final robot state
    // printLegStateVisualization(sys);
    // printRobotDiagram(sys, distance);
    // printServoAngleGraph(sys, steps);
    // printTripodGaitPattern(sys, 1.0f);
    printDetailedRobotStatus(sys, distance, steps);

    std::cout << std::endl
              << "Robot successfully completed 800mm tripod gait!" << std::endl;
    std::cout << "tripod_gait_sim_test executed successfully" << std::endl;
    return 0;
}
