#include "../src/locomotion_system.h"
#include "../src/pose_config.h"
#include "../src/pose_config_factory.h"
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
    std::cout << "HEXAPOD TRIPOD GAIT SIMULATION WITH STATE MACHINE" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Simulating 6-legged robot with 3DOF per leg" << std::endl;
    std::cout << "Distance: 800mm | Velocity: 400mm/s | Duration: 2s" << std::endl;
    std::cout << "Sensors: 6 FSR + 1 IMU | Total servos: 18" << std::endl;
    std::cout << "StateController: Full hierarchical state management" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;
}

// Forward declaration
static void printAngleBar(double angle, double min_angle, double max_angle);

// Add function to display robot dimensions
static void printRobotDimensions(const Parameters &p) {
    std::cout << "=========================================" << std::endl;
    std::cout << "           ROBOT DIMENSIONS" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "Hexagon Radius:    " << std::setw(6) << p.hexagon_radius << " mm" << std::endl;
    std::cout << "Coxa Length:       " << std::setw(6) << p.coxa_length << " mm" << std::endl;
    std::cout << "Femur Length:      " << std::setw(6) << p.femur_length << " mm" << std::endl;
    std::cout << "Tibia Length:      " << std::setw(6) << p.tibia_length << " mm" << std::endl;
    std::cout << "Robot Height:      " << std::setw(6) << p.robot_height << " mm" << std::endl;
    std::cout << "Height Offset:     " << std::setw(6) << p.height_offset << " mm" << std::endl;
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
    std::cout << " Phase | Status" << std::endl;

    std::cout << "----|";
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        std::cout << "------|------|------|";
    }
    std::cout << "-------|-------" << std::endl;
}

static void printAngles(int step, LocomotionSystem &sys, double phase) {
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

static void printRobotDiagram(LocomotionSystem &sys, double distance_covered) {
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

static void printAngleBar(double angle, double min_angle, double max_angle) {
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

static void printTripodGaitPattern(LocomotionSystem &sys, double phase) {
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

static void printDetailedRobotStatus(LocomotionSystem &sys, double distance_covered, int step) {
    std::cout << std::endl
              << "═══════════════════════════════════════" << std::endl;
    std::cout << "HEXAPOD STATUS - Step " << step << std::endl;
    std::cout << "═══════════════════════════════════════" << std::endl;

    // Motion status
    std::cout << "Distance: " << std::setw(6) << std::setprecision(1) << distance_covered
              << "mm / 800mm (" << std::setw(3) << (int)((distance_covered / 800.0f) * 100) << "%)" << std::endl;
    std::cout << "Step Height: " << std::setw(4) << std::setprecision(1) << sys.getStepHeight() << "mm" << std::endl;
    std::cout << "Step Length: " << std::setw(4) << std::setprecision(1) << sys.getStepLength() << "mm" << std::endl;

    // Robot dimensions
    const Parameters &p = sys.getParameters();
    std::cout << std::endl
              << "ROBOT DIMENSIONS:" << std::endl;
    std::cout << "Body Radius: " << std::setw(4) << std::setprecision(1) << p.hexagon_radius << "mm | ";
    std::cout << "Height: " << std::setw(4) << std::setprecision(1) << p.robot_height << "mm" << std::endl;
    std::cout << "Leg Segments: C" << std::setw(3) << p.coxa_length << "mm | ";
    std::cout << "F" << std::setw(3) << p.femur_length << "mm | ";
    std::cout << "T" << std::setw(3) << p.tibia_length << "mm" << std::endl;

    // Calculate total robot footprint
    double max_reach = p.coxa_length + p.femur_length + p.tibia_length;
    double footprint_radius = p.hexagon_radius + max_reach;
    std::cout << "Max Reach: " << std::setw(4) << std::setprecision(1) << max_reach << "mm | ";
    std::cout << "Footprint: " << std::setw(4) << std::setprecision(1) << (footprint_radius * 2) << "mm diameter" << std::endl;

    // Show active servo count
    int active_servos = 0;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        LegState state = sys.getLegState(leg);
        if (state == SWING_PHASE)
            active_servos += 3; // 3 servos per leg in swing
    }
    std::cout << "Active Servos: " << active_servos << "/18" << std::endl;
}

// State Controller status display functions
static void printStateControllerStatus(StateController &stateController, int step) {
    std::cout << "═══════════════════════════════════════" << std::endl;
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

    std::cout << "Posing Mode: ";
    switch (stateController.getPosingMode()) {
    case POSING_NONE:
        std::cout << "NONE";
        break;
    case POSING_X_Y:
        std::cout << "X_Y";
        break;
    case POSING_PITCH_ROLL:
        std::cout << "PITCH_ROLL";
        break;
    case POSING_Z_YAW:
        std::cout << "Z_YAW";
        break;
    case POSING_EXTERNAL:
        std::cout << "EXTERNAL";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;

    std::cout << "Cruise Control: ";
    switch (stateController.getCruiseControlMode()) {
    case CRUISE_CONTROL_OFF:
        std::cout << "OFF";
        break;
    case CRUISE_CONTROL_ON:
        std::cout << "ON";
        break;
    case CRUISE_CONTROL_EXTERNAL:
        std::cout << "EXTERNAL";
        break;
    default:
        std::cout << "INVALID";
        break;
    }
    std::cout << std::endl;

    std::cout << "Manual Legs: " << stateController.getManualLegCount() << std::endl;
    std::cout << "Transitioning: " << (stateController.isTransitioning() ? "YES" : "NO") << std::endl;
    std::cout << "Has Errors: " << (stateController.hasErrors() ? "YES" : "NO") << std::endl;
    std::cout << "Ready for Operation: " << (stateController.isReadyForOperation() ? "YES" : "NO") << std::endl;

    if (stateController.isTransitioning()) {
        TransitionProgress progress = stateController.getTransitionProgress();
        std::cout << "Transition Progress: " << progress.current_step << "/" << progress.total_steps
                  << " (" << progress.completion_percentage << "%)" << std::endl;
    }

    std::cout << "═══════════════════════════════════════" << std::endl;
}

static void printStateTransitionInfo(StateController &stateController, const std::string &action) {
    std::cout << "\n🔄 " << action << std::endl;
    if (stateController.isTransitioning()) {
        std::cout << "⏳ State transition in progress..." << std::endl;
        TransitionProgress progress = stateController.getTransitionProgress();
        std::cout << "Progress: " << progress.current_step << "/" << progress.total_steps
                  << " (" << progress.completion_percentage << "%)" << std::endl;
    } else {
        std::cout << "✓ State transition completed" << std::endl;
    }
}

int main() {
    printWelcome();

    // Test with different robot heights to demonstrate the height parameter effect
    std::cout << "Testing robot height parameter effect on servo angles..." << std::endl;
    std::cout << "=========================================" << std::endl;

    // Test configuration 1: Lower height (80mm)
    std::cout << "Configuration 1: Robot Height = 120mm" << std::endl;
    Parameters p1{};
    p1.hexagon_radius = 200;
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

    // Create pose configuration for first test system
    PoseConfiguration pose_config1 = getDefaultPoseConfig(p1);

    assert(sys1.initialize(&imu1, &fsr1, &servos1, pose_config1));
    assert(sys1.calibrateSystem());
    assert(sys1.setGaitType(TRIPOD_GAIT));
    assert(sys1.walkForward(400.0f));

    std::cout << "Sample joint angles at 100mm height:" << std::endl;
    std::cout << "=== About to call trajectory and setLegPosition ===" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D pos = sys1.calculateFootTrajectory(leg, 0.5f);
        std::cout << "  Leg " << leg + 1 << " target position: x=" << std::fixed << std::setprecision(1) << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
        sys1.setLegPosition(leg, pos);
        JointAngles q = sys1.getJointAngles(leg);
        std::cout << "  Leg " << leg + 1 << ": Coxa=" << std::fixed << std::setw(6) << std::setprecision(1) << q.coxa
                  << "° Femur=" << std::fixed << std::setw(6) << q.femur << "° Tibia=" << std::fixed << std::setw(6) << q.tibia << "°" << std::endl;

        // Verify forward kinematics result
        Point3D fk_pos = sys1.calculateForwardKinematics(leg, q);
        std::cout << "  FK verification: x=" << std::fixed << std::setprecision(1) << fk_pos.x << ", y=" << fk_pos.y << ", z=" << fk_pos.z << std::endl;
        double error = sqrt(pow(pos.x - fk_pos.x, 2) + pow(pos.y - fk_pos.y, 2) + pow(pos.z - fk_pos.z, 2));
        std::cout << "  IK error: " << std::fixed << std::setprecision(1) << error << "mm" << std::endl;
    }

    std::cout << std::endl;

    // Test configuration 2: Higher height (200mm)
    std::cout << "Configuration 2: Robot Height = 200mm" << std::endl;
    Parameters p2{};
    p2.hexagon_radius = 200;
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

    // Create pose configuration for second test system
    PoseConfiguration pose_config2 = getDefaultPoseConfig(p2);

    assert(sys2.initialize(&imu2, &fsr2, &servos2, pose_config2));
    assert(sys2.calibrateSystem());
    assert(sys2.setGaitType(TRIPOD_GAIT));
    assert(sys2.walkForward(400.0f));

    std::cout << "Sample joint angles at 200mm height:" << std::endl;
    std::cout << "=== About to call trajectory and setLegPosition ===" << std::endl;
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        Point3D pos = sys2.calculateFootTrajectory(leg, 0.5f);
        std::cout << "  Leg " << leg + 1 << " target position: x=" << std::fixed << std::setprecision(1) << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
        sys2.setLegPosition(leg, pos);
        JointAngles q = sys2.getJointAngles(leg);
        std::cout << "  Leg " << leg + 1 << ": Coxa=" << std::fixed << std::setw(6) << std::setprecision(1) << q.coxa
                  << "° Femur=" << std::fixed << std::setw(6) << q.femur << "° Tibia=" << std::fixed << std::setw(6) << q.tibia << "°" << std::endl;

        // Verify forward kinematics result
        Point3D fk_pos = sys2.calculateForwardKinematics(leg, q);
        std::cout << "  FK verification: x=" << std::fixed << std::setprecision(1) << fk_pos.x << ", y=" << fk_pos.y << ", z=" << fk_pos.z << std::endl;
        double error = sqrt(pow(pos.x - fk_pos.x, 2) + pow(pos.y - fk_pos.y, 2) + pow(pos.z - fk_pos.z, 2));
        std::cout << "  IK error: " << std::fixed << std::setprecision(1) << error << "mm" << std::endl;
    }

    std::cout << std::endl
              << "Notice how the servo angles differ between 100mm and 200mm heights!" << std::endl;
    std::cout << "=========================================" << std::endl
              << std::endl;

    // Initialize robot parameters for main simulation (use 150mm height)
    Parameters p{};
    p.hexagon_radius = 200;
    p.coxa_length = 50;
    p.femur_length = 101;
    p.tibia_length = 208;
    p.robot_height = 150;
    p.height_offset = 0;
    p.control_frequency = 50;
    // Use proper CSIRO-style joint limits
    p.coxa_angle_limits[0] = -65;
    p.coxa_angle_limits[1] = 65;
    p.femur_angle_limits[0] = -75;
    p.femur_angle_limits[1] = 75;
    p.tibia_angle_limits[0] = -45;
    p.tibia_angle_limits[1] = 45;

    // Display robot dimensions
    printRobotDimensions(p);

    // Simulation parameters - moved here to be available for state controller setup
    double velocity = 400.0f;              // mm/s
    double distance = 2400.0f;             // mm
    double duration = distance / velocity; // 2 seconds
    unsigned steps = static_cast<unsigned>(duration * p.control_frequency);

    // Initialize locomotion system
    LocomotionSystem sys(p);
    DummyIMU imu;
    DummyFSR fsr;
    DummyServo servos;

    // Create pose configuration for the robot
    PoseConfiguration pose_config = getDefaultPoseConfig(p);

    std::cout << "Initializing locomotion system..." << std::endl;
    assert(sys.initialize(&imu, &fsr, &servos, pose_config));
    assert(sys.calibrateSystem());

    // Test parallel sensor functionality
    // std::cout << "Testing parallel sensor updates..." << std::endl;
    // for (int i = 0; i < 5; i++) {
    //     bool sensor_update_success = sys.update();
    //     if (!sensor_update_success) {
    //         std::cout << "❌ ERROR: Parallel sensor update test failed on iteration " << i << std::endl;
    //         std::cout << "Error: " << sys.getErrorMessage(sys.getLastError()) << std::endl;
    //         return 1;
    //     }
    //     std::cout << "✓ Sensor update iteration " << i + 1 << " successful" << std::endl;
    // }
    // std::cout << "✅ Parallel sensor system working correctly" << std::endl;

    // Initialize State Controller
    std::cout << "Initializing State Controller..." << std::endl;
    StateMachineConfig stateConfig;
    stateConfig.max_manual_legs = 2;
    stateConfig.pack_unpack_time = 2.0f;
    stateConfig.enable_cruise_control = true;
    stateConfig.enable_manual_posing = true;
    stateConfig.transition_timeout = 10.0f;

    StateController stateController(sys, stateConfig);
    assert(stateController.initialize(pose_config));

    std::cout << "✓ State Controller initialized successfully" << std::endl;
    // printStateControllerStatus(stateController, 0);

    // Start state machine sequence
    // std::cout << "\n🚀 Starting State Machine Sequence..." << std::endl;

    // // 1. Request system operational
    // std::cout << "\n--- Phase 1: System Startup ---" << std::endl;
    stateController.requestSystemState(SYSTEM_OPERATIONAL);
    // for (int i = 0; i < 10; i++) {
    //     stateController.update(0.02f);
    //     if (!stateController.isTransitioning())
    //         break;
    // }
    // printStateTransitionInfo(stateController, "System operational requested");

    // 2. Request robot running
    // std::cout << "\n--- Phase 2: Robot State Transition ---" << std::endl;
    // stateController.requestRobotState(ROBOT_RUNNING);
    // for (int i = 0; i < 50; i++) {
    //     stateController.update(0.02f);
    //     if (!stateController.isTransitioning())
    //         break;
    // }
    // printStateTransitionInfo(stateController, "Robot running state requested");

    // 3. Set velocity for walking
    // std::cout << "\n--- Phase 3: Velocity Control Setup ---" << std::endl;
    Eigen::Vector2d linear_velocity(velocity, 0.0f);
    stateController.setDesiredVelocity(linear_velocity, 0.0f);
    // std::cout << "✓ Desired velocity set: " << velocity << " mm/s forward" << std::endl;

    // Setup gait after state controller is ready
    assert(sys.setStandingPose());
    assert(sys.setGaitType(TRIPOD_GAIT));
    assert(sys.walkForward(velocity));

    std::cout << "Starting tripod gait simulation with State Controller..." << std::endl;
    std::cout << "Total steps: " << steps << " | Step interval: " << (1000.0f / p.control_frequency) << "ms" << std::endl;
    std::cout << std::endl;

    // printStateControllerStatus(stateController, 0);

    // Track previous joint angles to detect changes
    JointAngles prev[NUM_LEGS];
    for (int i = 0; i < NUM_LEGS; ++i) {
        prev[i] = sys.getJointAngles(i);
    }

    bool changed = false;
    int successful_updates = 0;
    int failed_updates = 0;
    printHeader();

    for (unsigned s = 0; s < steps; ++s) {
        double phase = static_cast<double>(s) / static_cast<double>(steps);
        double distance_covered = phase * distance;
        double deltaTime = 1.0f / p.control_frequency;

        // Update State Controller first
        stateController.update(deltaTime);

        // Then update the locomotion system with error checking
        bool update_success = sys.update(deltaTime);
        if (!update_success) {
            failed_updates++;
            LocomotionSystem::ErrorCode last_error = sys.getLastError();
            std::cout << "⚠️ WARNING: Locomotion system update failed at step " << s << std::endl;
            std::cout << "Error Code: " << static_cast<int>(last_error) << std::endl;
            std::cout << "Error Message: " << sys.getErrorMessage(last_error) << std::endl;

            // For critical errors, we might want to abort the test
            if (last_error == LocomotionSystem::KINEMATICS_ERROR || last_error == LocomotionSystem::STABILITY_ERROR) {
                std::cout << "❌ CRITICAL ERROR: Aborting test due to critical failure" << std::endl;
                return 1;
            }

            // For non-critical errors, log and continue
            std::cout << "🔄 Continuing test despite non-critical error..." << std::endl;
        } else {
            successful_updates++;
        }

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

        // Show visual diagrams and state controller status every 25 steps
        // if (s % 25 == 0) {
        //     printStateControllerStatus(stateController, s);
        //     // printLegStateVisualization(sys);
        //     //  printRobotDiagram(sys, distance_covered);
        //     //  printServoAngleGraph(sys, s);
        //     //  printTripodGaitPattern(sys, phase);
        //     // printDetailedRobotStatus(sys, distance_covered, s);
        //     std::cout << std::endl;
        // }

        // Handle any state controller errors
        if (stateController.hasErrors()) {
            std::cout << "⚠️ State Controller Error detected at step " << s << std::endl;
            std::cout << "Error message: " << stateController.getLastErrorMessage().c_str() << std::endl;
            stateController.clearError();
        }

        // Simulate real-time execution - commented out for faster analysis
        // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    /*    // Final status
       std::cout << std::endl
                 << "=========================================" << std::endl;
       std::cout << "SIMULATION COMPLETED WITH STATE MACHINE" << std::endl;
       std::cout << "=========================================" << std::endl;

       // Parallel sensor system performance report
       std::cout << "\n📊 PARALLEL SENSOR SYSTEM PERFORMANCE:" << std::endl;
       std::cout << "Total update cycles: " << (successful_updates + failed_updates) << std::endl;
       std::cout << "Successful updates: " << successful_updates << " ("
                 << std::fixed << std::setprecision(1)
                 << (100.0f * successful_updates / (successful_updates + failed_updates)) << "%)" << std::endl;
       std::cout << "Failed updates: " << failed_updates << " ("
                 << std::fixed << std::setprecision(1)
                 << (100.0f * failed_updates / (successful_updates + failed_updates)) << "%)" << std::endl;

       if (failed_updates > 0) {
           std::cout << "⚠️ Warning: " << failed_updates << " sensor update failures detected" << std::endl;
       } else {
           std::cout << "✅ Perfect sensor update performance - no failures detected" << std::endl;
       }

       // Final State Controller status
       printStateControllerStatus(stateController, steps);

       if (!changed) {
           std::cout << "⚠️  Warning: servo angles did not change during simulation" << std::endl;
       } else {
           std::cout << "✓ Servo angles changed correctly during simulation" << std::endl;
       }

       // Test state machine functionality during simulation
       std::cout << "\n--- Testing State Machine Advanced Features ---" << std::endl;

       // Test emergency stop
       std::cout << "Testing emergency stop..." << std::endl;
       stateController.emergencyStop();
       stateController.update(0.02f);
       std::cout << "✓ Emergency stop executed" << std::endl;

       // Clear error and resume
       std::cout << "Clearing error and resuming..." << std::endl;
       stateController.clearError();
       stateController.requestSystemState(SYSTEM_OPERATIONAL);
       for (int i = 0; i < 10; i++) {
           stateController.update(0.02f);
           if (!stateController.isTransitioning())
               break;
       }
       std::cout << "✓ System resumed operational state" << std::endl;

       // Test pose control
       std::cout << "Testing pose control..." << std::endl;
       stateController.setPosingMode(POSING_X_Y);
       Eigen::Vector3d position(10.0f, 5.0f, 0.0f);
       Eigen::Vector3d orientation(0.0f, 0.0f, 0.1f);
       stateController.setDesiredPose(position, orientation);
       std::cout << "✓ X-Y posing mode set with desired pose" << std::endl;

       // Test cruise control
       std::cout << "Testing cruise control..." << std::endl;
       Eigen::Vector3d cruise_velocity(200.0f, 0.0f, 0.0f);
       stateController.setCruiseControlMode(CRUISE_CONTROL_ON, cruise_velocity);
       std::cout << "✓ Cruise control enabled" << std::endl;

       // Final robot state
       // printLegStateVisualization(sys);
       // printRobotDiagram(sys, distance);
       // printServoAngleGraph(sys, steps);
       // printTripodGaitPattern(sys, 1.0f);
       printDetailedRobotStatus(sys, distance, steps);

       // Final state controller status
       // printStateControllerStatus(stateController, steps + 1);

       std::cout << std::endl
                 << "🎉 Robot successfully completed 800mm tripod gait with State Controller!" << std::endl;
       std::cout << "✅ State Machine Integration: SUCCESSFUL" << std::endl;
       std::cout << "✅ Hierarchical State Management: VALIDATED" << std::endl;
       std::cout << "✅ Parallel Sensor System: " << (failed_updates == 0 ? "PERFECT" : "FUNCTIONAL") << std::endl;
       std::cout << "✅ Advanced Features: TESTED" << std::endl;

       if (failed_updates > 0) {
           std::cout << "⚠️ NOTE: " << failed_updates << " sensor failures occurred during test" << std::endl;
       }

       std::cout << "tripod_gait_sim_test with StateController and Parallel Sensors executed successfully" << std::endl; */
    return 0;
}
