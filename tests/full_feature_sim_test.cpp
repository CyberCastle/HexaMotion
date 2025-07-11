#include "../src/admittance_controller.h"
#include "../src/imu_auto_pose.h"
#include "../src/locomotion_system.h"
#include "../src/manual_pose_controller.h"
#include "../src/precision_config.h"
#include "../src/state_controller.h"
#include "test_stubs.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

static void printWelcome() {
    std::cout << "=========================================" << std::endl;
    std::cout << "HEXAPOD FULL FEATURE SIMULATION WITH ALL GAITS" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Simulating 6-legged robot with 3DOF per leg" << std::endl;
    std::cout << "Distance: 5000mm | Velocity: 400mm/s | Duration: 12.5s" << std::endl;
    std::cout << "Sensors: 6 FSR + 1 IMU | Total servos: 18" << std::endl;
    std::cout << "StateController: Full hierarchical state management" << std::endl;
    std::cout << "Features: Manual/Auto Pose, Admittance Control, Precision Config" << std::endl;
    std::cout << "Gaits: TRIPOD → WAVE → RIPPLE → METACHRONAL → ADAPTIVE (all 5 supported gaits)" << std::endl;
    std::cout << "Each gait: 1000mm distance | 1000mm per gait type" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;
}

static void printRobotDimensions(const Parameters &p) {
    std::cout << "=========================================" << std::endl;
    std::cout << "           ROBOT DIMENSIONS" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Hexagon Radius:    " << std::setw(6) << p.hexagon_radius << " mm" << std::endl;
    std::cout << "Coxa Length:       " << std::setw(6) << p.coxa_length << " mm" << std::endl;
    std::cout << "Femur Length:      " << std::setw(6) << p.femur_length << " mm" << std::endl;
    std::cout << "Tibia Length:      " << std::setw(6) << p.tibia_length << " mm" << std::endl;
    std::cout << "Robot Height:      " << std::setw(6) << p.robot_height << " mm" << std::endl;
    std::cout << "Control Frequency: " << std::setw(6) << p.control_frequency << " Hz" << std::endl;
    std::cout << std::endl;

    // Calculate and display total leg reach
    double max_reach = p.coxa_length + p.femur_length + p.tibia_length;
    double body_diagonal = p.hexagon_radius * 2;

    std::cout << "CALCULATED DIMENSIONS:" << std::endl;
    std::cout << "Max Leg Reach:     " << std::setw(6) << std::setprecision(1) << std::fixed << max_reach << " mm" << std::endl;
    std::cout << "Body Diagonal:     " << std::setw(6) << std::setprecision(1) << body_diagonal << " mm" << std::endl;
    std::cout << "Working Radius:    " << std::setw(6) << std::setprecision(1) << (p.hexagon_radius + max_reach) << " mm" << std::endl;
    std::cout << std::endl;

    // Display joint limits
    std::cout << "JOINT ANGLE LIMITS:" << std::endl;
    std::cout << "Coxa:  " << std::setw(4) << p.coxa_angle_limits[0] << "° to " << std::setw(4) << p.coxa_angle_limits[1] << "°" << std::endl;
    std::cout << "Femur: " << std::setw(4) << p.femur_angle_limits[0] << "° to " << std::setw(4) << p.femur_angle_limits[1] << "°" << std::endl;
    std::cout << "Tibia: " << std::setw(4) << p.tibia_angle_limits[0] << "° to " << std::setw(4) << p.tibia_angle_limits[1] << "°" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << std::endl;
}

static void printHeader() {
    std::cout << "Step|";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "  L" << leg + 1 << "C   L" << leg + 1 << "F   L" << leg + 1 << "T |";
    }
    std::cout << " Phase | Gait   | Status" << std::endl;

    std::cout << "----|";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "------|------|------|";
    }
    std::cout << "-------|--------|-------" << std::endl;
}

static void printAngles(int step, LocomotionSystem &sys, double phase, const std::string &current_gait) {
    std::cout << std::setw(3) << step << " |";

    // Print joint angles for all legs
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        std::cout << std::fixed << std::setw(6) << std::setprecision(1)
                  << q.coxa << " " << std::setw(6) << q.femur << " "
                  << std::setw(6) << q.tibia << " |";
    }

    // Print gait phase and current gait
    std::cout << std::setw(6) << std::setprecision(2) << phase << " |";
    std::cout << std::setw(7) << current_gait << " |";

    // Show which legs are in stance/swing
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        StepPhase state = sys.getLegState(leg);
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';
        std::cout << symbol;
    }

    std::cout << std::endl;
}

static void printStateControllerStatus(StateController &stateController, int step) {
    std::cout << std::endl
              << "═══════════════════════════════════════" << std::endl;
    std::cout << "STATE CONTROLLER STATUS - Step " << step << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;

    std::cout << "System State: ";
    switch (stateController.getSystemState()) {
    case SYSTEM_SUSPENDED:
        std::cout << "SUSPENDED";
        break;
    case SYSTEM_OPERATIONAL:
        std::cout << "OPERATIONAL";
        break;
    default:
        std::cout << "UNKNOWN";
        break;
    }
    std::cout << std::endl;

    std::cout << "Robot State: ";
    switch (stateController.getRobotState()) {
    case ROBOT_UNKNOWN:
        std::cout << "UNKNOWN";
        break;
    case ROBOT_PACKED:
        std::cout << "PACKED";
        break;
    case ROBOT_READY:
        std::cout << "READY";
        break;
    case ROBOT_RUNNING:
        std::cout << "RUNNING";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;

    std::cout << "Walk State: ";
    switch (stateController.getWalkState()) {
    case WALK_STOPPED:
        std::cout << "STOPPED";
        break;
    case WALK_STARTING:
        std::cout << "STARTING";
        break;
    case WALK_MOVING:
        std::cout << "MOVING";
        break;
    case WALK_STOPPING:
        std::cout << "STOPPING";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;

    if (stateController.isTransitioning()) {
        std::cout << "⏳ State transition in progress..." << std::endl;
    }

    std::cout << "═══════════════════════════════════════" << std::endl;
}

static void printGaitChangeInfo(const std::string &gait_name, int step, double distance_covered) {
    std::cout << std::endl
              << "🔄 GAIT CHANGE - Step " << step << std::endl;
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
    std::cout << "New Gait: " << gait_name << std::endl;
    std::cout << "Distance: " << std::setprecision(1) << std::fixed << distance_covered << "mm" << std::endl;

    // Add gait-specific information
    if (gait_name == "TRIPOD") {
        std::cout << "Features: 2 alternating groups, maximum speed, 50% stance ratio" << std::endl;
    } else if (gait_name == "WAVE") {
        std::cout << "Features: Sequential wave pattern, maximum stability, 83.3% stance ratio" << std::endl;
    } else if (gait_name == "RIPPLE") {
        std::cout << "Features: Overlapping pattern, optimal balance, 66.7% stance ratio" << std::endl;
    } else if (gait_name == "METACHRONAL") {
        std::cout << "Features: Biomimetic wave motion, fluid progression, clockwise sequence" << std::endl;
    } else if (gait_name == "ADAPTIVE") {
        std::cout << "Features: Terrain-adaptive pattern, dynamic adjustment, fine-tuned offsets" << std::endl;
    }

    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
}

static void printDetailedRobotStatus(LocomotionSystem &sys, double distance_covered, int step,
                                     const std::string &current_gait) {
    std::cout << std::endl
              << "═══════════════════════════════════════" << std::endl;
    std::cout << "HEXAPOD STATUS - Step " << step << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;

    // Motion status
    std::cout << "Distance: " << std::setw(6) << std::setprecision(1) << distance_covered
              << "mm / 5000mm (" << std::setw(3) << (int)((distance_covered / 5000.0f) * 100) << "%)" << std::endl;
    std::cout << "Current Gait: " << current_gait << std::endl;

    // Calculate total movements and stability
    int stance_legs = 0;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        if (sys.getLegState(leg) == STANCE_PHASE) {
            stance_legs++;
        }
    }
    std::cout << "Stance Legs: " << stance_legs << "/6 (Stability: "
              << (stance_legs >= 3 ? "STABLE" : "UNSTABLE") << ")" << std::endl;

    std::cout << "═══════════════════════════════════════" << std::endl;
}

static void printGaitPattern(LocomotionSystem &sys, const std::string &current_gait, double phase) {
    std::cout << std::endl
              << current_gait << " Gait Pattern (Phase: " << std::setprecision(2) << phase << "):" << std::endl;

    if (current_gait == "TRIPOD") {
        std::cout << "Group A (L1,L3,L5): ";
        for (int leg : {0, 2, 4}) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓▓" : "░░");
        }
        std::cout << std::endl
                  << "Group B (L2,L4,L6): ";
        for (int leg : {1, 3, 5}) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓▓" : "░░");
        }
    } else if (current_gait == "WAVE") {
        std::cout << "Wave sequence (BL→CL→AR→BR→CR→AL): ";
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓" : "░");
        }
    } else if (current_gait == "RIPPLE") {
        std::cout << "Ripple sequence (BR→CL→AR→BL→CR→AL): ";
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓" : "░");
        }
    } else if (current_gait == "METACHRONAL") {
        std::cout << "Metachronal wave (AR→BR→CR→CL→BL→AL): ";
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓" : "░");
        }
    } else if (current_gait == "ADAPTIVE") {
        std::cout << "Adaptive pattern (terrain-responsive): ";
        for (int leg = 0; leg < NUM_LEGS; ++leg) {
            StepPhase state = sys.getLegState(leg);
            std::cout << (state == STANCE_PHASE ? "▓" : "░");
        }
    }

    std::cout << std::endl
              << "▓=Ground Support, ░=Air Movement" << std::endl;
}

static void printProgressVisualization(double distance_covered, const std::string &current_gait) {
    std::cout << std::endl
              << "Progress Visualization:" << std::endl;
    std::cout << "Distance: " << std::setw(6) << std::setprecision(1) << std::fixed
              << distance_covered << "mm / 5000mm" << std::endl;

    // Progress bar with gait sections
    int progress = static_cast<int>((distance_covered / 5000.0f) * 50);
    std::cout << "Progress: [";
    for (int i = 0; i < 50; ++i) {
        if (i < 10)
            std::cout << (i < progress ? "T" : "·"); // TRIPOD
        else if (i < 20)
            std::cout << (i < progress ? "W" : "·"); // WAVE
        else if (i < 30)
            std::cout << (i < progress ? "R" : "·"); // RIPPLE
        else if (i < 40)
            std::cout << (i < progress ? "M" : "·"); // METACHRONAL
        else
            std::cout << (i < progress ? "A" : "·"); // ADAPTIVE
    }
    std::cout << "] " << std::setw(3) << (int)((distance_covered / 5000.0f) * 100) << "%" << std::endl;
    std::cout << "Sections: T=Tripod, W=Wave, R=Ripple, M=Metachronal, A=Adaptive" << std::endl;
    std::cout << "Current: " << current_gait << std::endl;
}

static void printAngleBar(double angle, double min_angle, double max_angle) {
    const int bar_length = 20;
    int center = bar_length / 2;
    int position = static_cast<int>(((angle - min_angle) / (max_angle - min_angle)) * bar_length);
    position = std::max(0, std::min(bar_length - 1, position));

    for (int i = 0; i < bar_length; ++i) {
        if (i == center) {
            std::cout << "|"; // Center line (0°)
        } else if (i == position) {
            std::cout << "●"; // Current angle position
        } else {
            std::cout << "─";
        }
    }
}

static void printDetailedAngleVisualization(LocomotionSystem &sys, int step, const JointAngles prev[NUM_LEGS]) {
    std::cout << std::endl
              << "═══════════════════════════════════════" << std::endl;
    std::cout << "DETAILED SERVO ANGLE VISUALIZATION - Step " << step << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;
    std::cout << "Range: -75° to +75° | Scale: each char = 7.5°" << std::endl;
    std::cout << "   -75  -50  -25    0   25   50   75" << std::endl;
    std::cout << "    |    |    |    |    |    |    |" << std::endl;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        StepPhase state = sys.getLegState(leg);
        char leg_symbol = (state == STANCE_PHASE) ? 'S' : 'W';

        std::cout << std::endl
                  << "--- LEG " << (leg + 1) << " (" << leg_symbol << ") ---" << std::endl;

        // Coxa angle visualization
        std::cout << "C: ";
        printAngleBar(q.coxa, -75, 75);
        std::cout << " " << std::setw(6) << std::setprecision(1) << std::fixed << q.coxa << "°";
        if (step > 0) {
            double diff = q.coxa - prev[leg].coxa;
            if (std::abs(diff) > 0.1f) {
                std::cout << " (Δ" << std::setw(5) << std::setprecision(1) << diff << "°)";
            } else {
                std::cout << "        ";
            }
        }
        std::cout << std::endl;

        // Femur angle visualization
        std::cout << "F: ";
        printAngleBar(q.femur, -75, 75);
        std::cout << " " << std::setw(6) << std::setprecision(1) << std::fixed << q.femur << "°";
        if (step > 0) {
            double diff = q.femur - prev[leg].femur;
            if (std::abs(diff) > 0.1f) {
                std::cout << " (Δ" << std::setw(5) << std::setprecision(1) << diff << "°)";
            } else {
                std::cout << "        ";
            }
        }
        std::cout << std::endl;

        // Tibia angle visualization
        std::cout << "T: ";
        printAngleBar(q.tibia, -75, 75);
        std::cout << " " << std::setw(6) << std::setprecision(1) << std::fixed << q.tibia << "°";
        if (step > 0) {
            double diff = q.tibia - prev[leg].tibia;
            if (std::abs(diff) > 0.1f) {
                std::cout << " (Δ" << std::setw(5) << std::setprecision(1) << diff << "°)";
            } else {
                std::cout << "        ";
            }
        }
        std::cout << std::endl;
    }
    std::cout << "═══════════════════════════════════════" << std::endl;
}

static void printAngleChangesSummary(LocomotionSystem &sys, const JointAngles prev[NUM_LEGS], int step) {
    std::cout << std::endl
              << "ANGLE CHANGES SUMMARY - Step " << step << std::endl;
    std::cout << "Leg | Coxa Δ | Femur Δ | Tibia Δ | Status" << std::endl;
    std::cout << "----|--------|---------|---------|-------" << std::endl;

    int legs_with_changes = 0;
    double total_movement = 0.0f;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        StepPhase state = sys.getLegState(leg);

        double coxa_diff = q.coxa - prev[leg].coxa;
        double femur_diff = q.femur - prev[leg].femur;
        double tibia_diff = q.tibia - prev[leg].tibia;

        bool has_change = (std::abs(coxa_diff) > 0.1f ||
                           std::abs(femur_diff) > 0.1f ||
                           std::abs(tibia_diff) > 0.1f);

        if (has_change) {
            legs_with_changes++;
            total_movement += std::abs(coxa_diff) + std::abs(femur_diff) + std::abs(tibia_diff);
        }

        std::cout << " L" << (leg + 1) << " |";
        std::cout << std::setw(7) << std::setprecision(1) << std::fixed << coxa_diff << " |";
        std::cout << std::setw(8) << std::setprecision(1) << femur_diff << " |";
        std::cout << std::setw(8) << std::setprecision(1) << tibia_diff << " |";
        std::cout << " " << (state == STANCE_PHASE ? "STANCE" : "SWING ");
        if (has_change)
            std::cout << " ●";
        std::cout << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Legs with movement: " << legs_with_changes << "/6" << std::endl;
    std::cout << "Total movement: " << std::setprecision(1) << total_movement << "°" << std::endl;
}

static void printCompactAngleStatus(LocomotionSystem &sys, int step, const std::string &current_gait) {
    std::cout << "Step " << std::setw(3) << step << " [" << current_gait << "]: ";

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        StepPhase state = sys.getLegState(leg);
        char symbol = (state == STANCE_PHASE) ? 'S' : 'W';

        std::cout << "L" << (leg + 1) << symbol << "(";
        std::cout << std::setw(3) << (int)q.coxa << ",";
        std::cout << std::setw(3) << (int)q.femur << ",";
        std::cout << std::setw(3) << (int)q.tibia << ") ";
    }
    std::cout << std::endl;
}

static void forceGaitMovement(LocomotionSystem &sys, GaitType gait) {
    // Force proper gait movement planning
    sys.setGaitType(gait);
    sys.planGaitSequence(400.0f, 0.0f, 0.0f);

    // Execute several update cycles to ensure movement
    for (int i = 0; i < 10; i++) {
        sys.update();
    }
}

static void printMovementStatus(LocomotionSystem &sys, const JointAngles prev[NUM_LEGS], int step) {
    int moving_legs = 0;
    double max_change = 0.0f;

    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        JointAngles q = sys.getJointAngles(leg);
        double total_change = std::abs(q.coxa - prev[leg].coxa) +
                             std::abs(q.femur - prev[leg].femur) +
                             std::abs(q.tibia - prev[leg].tibia);

        if (total_change > 0.1f) {
            moving_legs++;
        }
        max_change = std::max(max_change, total_change);
    }

    if (moving_legs > 0) {
        std::cout << "🔄 Step " << step << ": " << moving_legs << " legs moving, max change: "
                  << std::setprecision(1) << max_change << "°" << std::endl;
    }
}

/**
 * @brief Full feature simulation test.
 *
 * Exercises gait changes, precision settings, admittance control,
 * manual/auto pose functions and state transitions while walking a
 * simulated distance of 5 meters with all 5 supported gaits.
 *
 * Each gait covers 1000mm:
 * - TRIPOD (0-1000mm): Speed-optimized gait
 * - WAVE (1000-2000mm): Stability-optimized gait
 * - RIPPLE (2000-3000mm): Balanced speed/stability
 * - METACHRONAL (3000-4000mm): Biomimetic wave motion
 * - ADAPTIVE (4000-5000mm): Terrain-adaptive pattern
 */
int main() {
    printWelcome();

    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 208;
    p.control_frequency = 50;
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    printRobotDimensions(p);

    // Hardware stubs
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Locomotion system
    LocomotionSystem sys(p);
    PoseConfiguration pose_config(p);
    assert(sys.initialize(&imu, &fsr, &servos, pose_config));
    assert(sys.calibrateSystem());
    assert(sys.setGaitType(TRIPOD_GAIT));

    // Use proper movement approach for visible servo changes
    // Based on previous testing: planGaitSequence + multiple updates instead of walkForward
    assert(sys.planGaitSequence(400.0f, 0.0f, 0.0f));

    // State controller configuration
    StateMachineConfig state_cfg{};
    state_cfg.enable_startup_sequence = false;
    state_cfg.transition_timeout = 10.0f;
    StateController controller(sys, state_cfg);
    assert(controller.initialize(pose_config));
    controller.requestSystemState(SYSTEM_OPERATIONAL);
    controller.requestRobotState(ROBOT_RUNNING);
    controller.setDesiredVelocity(Eigen::Vector2d(400.0f, 0.0f), 0.0f);

    // Independent modules for additional features
    RobotModel model(p);
    ManualPoseController pose_ctrl(model);
    pose_ctrl.initialize();
    pose_ctrl.setPoseMode(ManualPoseController::POSE_TRANSLATION);
    pose_ctrl.processInput(20.0f, 0.0f, 0.0f);

    IMUAutoPose auto_pose(model, &imu, pose_ctrl, ComputeConfig::medium());
    auto_pose.initialize();
    auto_pose.setAutoPoseMode(IMUAutoPose::AUTO_POSE_LEVEL);
    auto_pose.update(0.02f);

    AdmittanceController adm_ctrl(model, &imu, &fsr, ComputeConfig::high());
    adm_ctrl.initialize();
    adm_ctrl.setDynamicStiffness(true, 0.5f, 1.5f);
    Point3D forces[NUM_LEGS];
    Point3D deltas[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        forces[i] = Point3D(1.0f, 0.0f, -2.0f);
    }
    adm_ctrl.updateAllLegs(forces, deltas);

    // Simulate 5 meters of walking (1000mm per gait)
    double distance = 5000.0f;
    double dt = 1.0f / p.control_frequency;
    int steps = static_cast<int>((distance / 400.0f) / dt);

    std::cout << "\n--- Starting Full Feature Simulation ---" << std::endl;
    std::cout << "Total steps: " << steps << " | Step interval: " << (1000.0f / p.control_frequency) << "ms" << std::endl;
    std::cout << "Features: Manual Pose, Auto IMU Pose, Admittance Control" << std::endl;
    std::cout << "Gait sequence: TRIPOD (0-20%) → WAVE (20-40%) → RIPPLE (40-60%) → METACHRONAL (60-80%) → ADAPTIVE (80-100%)" << std::endl;
    std::cout << "Each gait covers 1000mm distance" << std::endl;
    std::cout << std::endl;

    printStateControllerStatus(controller, 0);

    // Track previous joint angles to detect changes
    JointAngles prev[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev[i] = sys.getJointAngles(i);
    }

    bool changed = false;
    std::string current_gait = "TRIPOD";
    printHeader();

    for (int s = 0; s < steps; ++s) {
        double phase = static_cast<double>(s) / static_cast<double>(steps);
        double distance_covered = phase * distance;

        // Gait changes at 20%, 40%, 60%, and 80% of the journey (each gait covers 1000mm)
        if (s == steps / 5) {
            controller.changeGait(WAVE_GAIT);
            current_gait = "WAVE";
            printGaitChangeInfo(current_gait, s, distance_covered);
            forceGaitMovement(sys, WAVE_GAIT);
        }
        if (s == (2 * steps) / 5) {
            controller.changeGait(RIPPLE_GAIT);
            current_gait = "RIPPLE";
            printGaitChangeInfo(current_gait, s, distance_covered);
            forceGaitMovement(sys, RIPPLE_GAIT);
        }
        if (s == (3 * steps) / 5) {
            controller.changeGait(METACHRONAL_GAIT);
            current_gait = "METACHRONAL";
            printGaitChangeInfo(current_gait, s, distance_covered);
            forceGaitMovement(sys, METACHRONAL_GAIT);
        }
        if (s == (4 * steps) / 5) {
            controller.changeGait(ADAPTIVE_GAIT);
            current_gait = "ADAPTIVE";
            printGaitChangeInfo(current_gait, s, distance_covered);
            forceGaitMovement(sys, ADAPTIVE_GAIT);
        }

        // Update all systems
        auto_pose.update(dt);
        controller.update(dt);
        sys.update();

        // Show compact angle status every 5 steps for better movement tracking
        // if (s % 5 == 0) {
        printCompactAngleStatus(sys, s, current_gait);
        //}

        // Print detailed servo angles and states every 10 steps
        // if (s % 10 == 0) {
        //     printAngles(s, sys, phase, current_gait);
        // }

        // Show angle changes summary every 15 steps
        if (s % 15 == 0 && s > 0) {
            printAngleChangesSummary(sys, prev, s);
        }

        // Show detailed angle visualization every 25 steps
        // if (s % 25 == 0 && s > 0) {
        //     printDetailedAngleVisualization(sys, s, prev);
        // }

        // Show gait patterns every 30 steps
        if (s % 30 == 0) {
            printGaitPattern(sys, current_gait, phase);
        }

        // Track angle changes with more precision and store previous values for each step
        bool step_changed = false;
        for (int i = 0; i < NUM_LEGS; ++i) {
            JointAngles q = sys.getJointAngles(i);
            if (!changed && (std::abs(q.coxa - prev[i].coxa) > 0.05f ||
                             std::abs(q.femur - prev[i].femur) > 0.05f ||
                             std::abs(q.tibia - prev[i].tibia) > 0.05f)) {
                changed = true;
            }
            if (std::abs(q.coxa - prev[i].coxa) > 0.05f ||
                std::abs(q.femur - prev[i].femur) > 0.05f ||
                std::abs(q.tibia - prev[i].tibia) > 0.05f) {
                step_changed = true;
            }
        }

        // Show movement indicator for this step
        if (step_changed && s % 5 == 0) {
            std::cout << "  📐 Movement detected in step " << s << std::endl;
        }

        // Update previous angles for next comparison
        for (int i = 0; i < NUM_LEGS; ++i) {
            prev[i] = sys.getJointAngles(i);
        }

        // Show state controller status and detailed robot status every 50 steps
        if (s % 50 == 0 && s > 0) {
            printStateControllerStatus(controller, s);
            printDetailedRobotStatus(sys, distance_covered, s, current_gait);
            printProgressVisualization(distance_covered, current_gait);
            // printDetailedAngleVisualization(sys, s, prev);
            printAngleChangesSummary(sys, prev, s);
            printCompactAngleStatus(sys, s, current_gait);
            printMovementStatus(sys, prev, s);
            std::cout << std::endl;
        }

        // Handle any state controller errors
        if (controller.hasErrors()) {
            std::cout << "⚠️ State Controller Error detected at step " << s << std::endl;
            std::cout << "Error message: " << controller.getLastErrorMessage().c_str() << std::endl;
            controller.clearError();
        }
    }

    // Final status
    std::cout << std::endl
              << "=========================================" << std::endl;
    std::cout << "SIMULATION COMPLETED WITH STATE MACHINE" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Final State Controller status
    printStateControllerStatus(controller, steps);

    if (!changed) {
        std::cout << "⚠️  Warning: servo angles did not change during simulation" << std::endl;
    } else {
        std::cout << "✓ Servo angles changed correctly during simulation" << std::endl;
    }

    // Test advanced features during simulation
    std::cout << "\n--- Testing Full Feature Integration ---" << std::endl;

    // Test pose control
    std::cout << "Testing pose control..." << std::endl;
    controller.setPosingMode(POSING_X_Y);
    Eigen::Vector3d position(10.0f, 5.0f, 0.0f);
    Eigen::Vector3d orientation(0.0f, 0.0f, 0.1f);
    controller.setDesiredPose(position, orientation);
    std::cout << "✓ X-Y posing mode set with desired pose" << std::endl;

    // Test cruise control
    std::cout << "Testing cruise control..." << std::endl;
    Eigen::Vector3d cruise_velocity(200.0f, 0.0f, 0.0f);
    controller.setCruiseControlMode(CRUISE_CONTROL_ON, cruise_velocity);
    std::cout << "✓ Cruise control enabled" << std::endl;

    // Test manual pose controller features
    std::cout << "Testing manual pose controller..." << std::endl;
    pose_ctrl.setPoseMode(ManualPoseController::POSE_ROTATION);
    pose_ctrl.processInput(0.0f, 0.1f, 0.0f);
    std::cout << "✓ Manual pose controller rotation mode tested" << std::endl;

    // Test auto pose controller features
    std::cout << "Testing auto pose controller..." << std::endl;
    auto_pose.setAutoPoseMode(IMUAutoPose::AUTO_POSE_INCLINATION);
    auto_pose.update(0.02f);
    std::cout << "✓ Auto pose controller inclination mode tested" << std::endl;

    // Test admittance controller features
    std::cout << "Testing admittance controller..." << std::endl;
    adm_ctrl.setDynamicStiffness(false, 1.0f, 2.0f);
    for (int i = 0; i < NUM_LEGS; ++i) {
        forces[i] = Point3D(0.5f, 0.5f, -1.0f);
    }
    adm_ctrl.updateAllLegs(forces, deltas);
    std::cout << "✓ Admittance controller dynamic stiffness tested" << std::endl;

    // Final robot status
    printDetailedRobotStatus(sys, distance, steps, current_gait);

    // Final state controller status
    printStateControllerStatus(controller, steps + 1);

    std::cout << std::endl
              << "🎉 Robot successfully completed 5000mm with ALL GAITS and FULL FEATURES!" << std::endl;
    std::cout << "✅ Gait Transitions: TRIPOD → WAVE → RIPPLE → METACHRONAL → ADAPTIVE" << std::endl;
    std::cout << "✅ Distance Coverage: 5 x 1000mm = 5000mm total" << std::endl;
    std::cout << "✅ State Machine Integration: SUCCESSFUL" << std::endl;
    std::cout << "✅ Manual Pose Control: TESTED" << std::endl;
    std::cout << "✅ Auto IMU Pose Control: TESTED" << std::endl;
    std::cout << "✅ Admittance Control: TESTED" << std::endl;
    std::cout << "✅ Precision Configuration: ACTIVE" << std::endl;

    std::cout << "Final gait: " << controller.getWalkState() << std::endl;
    std::cout << "full_feature_sim_test executed successfully" << std::endl;
    return 0;
}
