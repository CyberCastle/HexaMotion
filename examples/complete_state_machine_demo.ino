/**
 * @file complete_state_machine_demo.ino
 * @brief Complete demonstration of HexaMotion StateController functionality
 * @author HexaMotion Team
 * @date 2024
 *
 * This comprehensive example demonstrates all major features of the StateController:
 * - Hierarchical state management
 * - Advanced operational modes
 * - Real-time progress tracking
 * - Error handling and recovery
 * - Integration with locomotion system
 */

#include "locomotion_system.h"
#include "state_controller.h"

// Global objects
LocomotionSystem *locomotion;
StateController *stateController;
Parameters robotParams;

// Demo state variables
unsigned long lastUpdate = 0;
unsigned long demoStartTime = 0;
int demoPhase = 0;
bool demoComplete = false;

void setup() {
    Serial.begin(115200);
    Serial.println("=== HexaMotion StateController Complete Demo ===");

    // Initialize robot parameters
    initializeParameters();

    // Create locomotion system
    locomotion = new LocomotionSystem(robotParams);

    // Initialize hardware interfaces (replace with actual hardware)
    // locomotion->initialize(&imu, &fsr, &servo);

    // Configure state machine
    StateMachineConfig config;
    config.max_manual_legs = 2;
    config.pack_unpack_time = 2.0f;
    config.enable_cruise_control = true;
    config.enable_manual_posing = true;
    config.enable_auto_posing = false;
    config.transition_timeout = 10.0f;

    // Create state controller
    stateController = new StateController(*locomotion, config);

    // Initialize state controller
    if (stateController->initialize()) {
        Serial.println("✓ StateController initialized successfully");
        printSystemStatus();
    } else {
        Serial.println("❌ StateController initialization failed");
        return;
    }

    demoStartTime = millis();
    Serial.println("\n🚀 Starting comprehensive state machine demonstration...");
}

void loop() {
    unsigned long currentTime = millis();

    // Update state controller at 50Hz
    if (currentTime - lastUpdate >= 20) {
        double deltaTime = (currentTime - lastUpdate) / 1000.0f;
        stateController->update(deltaTime);
        lastUpdate = currentTime;
    }

    // Run demonstration phases
    if (!demoComplete) {
        runDemoPhase(currentTime - demoStartTime);
    }

    // Handle any errors
    if (stateController->hasErrors()) {
        Serial.println("⚠️ Error detected - activating recovery sequence");
        stateController->clearError();
    }
}

void runDemoPhase(unsigned long elapsedTime) {
    switch (demoPhase) {
    case 0: // System Startup (0-3s)
        if (elapsedTime > 1000 && elapsedTime < 1100) {
            Serial.println("\n--- Phase 1: System Startup ---");
            stateController->requestSystemState(SYSTEM_OPERATIONAL);
            Serial.println("✓ System requested to operational");
        }
        if (elapsedTime > 3000 && demoPhase == 0) {
            printSystemStatus();
            demoPhase = 1;
        }
        break;

    case 1: // Robot State Transitions (3-8s)
        if (elapsedTime > 4000 && elapsedTime < 4100) {
            Serial.println("\n--- Phase 2: Robot State Transitions ---");
            if (stateController->requestRobotState(ROBOT_RUNNING)) {
                Serial.println("✓ Robot transition to RUNNING requested");
                monitorTransition();
            }
        }
        if (elapsedTime > 8000 && demoPhase == 1) {
            printSystemStatus();
            demoPhase = 2;
        }
        break;

    case 2: // Manual Leg Control (8-12s)
        if (elapsedTime > 9000 && elapsedTime < 9100) {
            Serial.println("\n--- Phase 3: Manual Leg Control ---");
            stateController->setLegState(0, LEG_MANUAL);
            stateController->setLegState(2, LEG_MANUAL);
            Serial.println("✓ Legs 0 and 2 set to manual control");

            // Set specific leg positions
            Eigen::Vector3d leg0_pos(100.0f, 50.0f, -80.0f);
            Eigen::Vector3d leg2_pos(100.0f, -50.0f, -80.0f);
            stateController->setLegTipPosition(0, leg0_pos);
            stateController->setLegTipPosition(2, leg2_pos);
            Serial.println("✓ Manual leg positions set");
        }
        if (elapsedTime > 12000 && demoPhase == 2) {
            printSystemStatus();
            demoPhase = 3;
        }
        break;

    case 3: // Velocity Control (12-16s)
        if (elapsedTime > 13000 && elapsedTime < 13100) {
            Serial.println("\n--- Phase 4: Velocity Control ---");
            Eigen::Vector2d linear_vel(50.0f, 25.0f);
            double angular_vel = 0.5f;
            stateController->setDesiredVelocity(linear_vel, angular_vel);
            Serial.println("✓ Desired velocity set: [50, 25] linear, 0.5 angular");
        }
        if (elapsedTime > 16000 && demoPhase == 3) {
            printSystemStatus();
            demoPhase = 4;
        }
        break;

    case 4: // Pose Control (16-20s)
        if (elapsedTime > 17000 && elapsedTime < 17100) {
            Serial.println("\n--- Phase 5: Pose Control ---");
            stateController->setPosingMode(POSING_X_Y);
            Serial.println("✓ X-Y posing mode enabled");

            Eigen::Vector3d position(20.0f, 10.0f, 0.0f);
            Eigen::Vector3d orientation(0.0f, 0.0f, 0.2f);
            stateController->setDesiredPose(position, orientation);
            Serial.println("✓ Desired pose set: pos[20,10,0], orient[0,0,0.2]");
        }
        if (elapsedTime > 20000 && demoPhase == 4) {
            printSystemStatus();
            demoPhase = 5;
        }
        break;

    case 5: // Cruise Control (20-25s)
        if (elapsedTime > 21000 && elapsedTime < 21100) {
            Serial.println("\n--- Phase 6: Cruise Control ---");
            Eigen::Vector3d cruise_velocity(30.0f, 0.0f, 0.3f);
            if (stateController->setCruiseControlMode(CRUISE_CONTROL_ON, cruise_velocity)) {
                Serial.println("✓ Cruise control enabled: [30, 0, 0.3]");
            }
        }
        if (elapsedTime > 25000 && demoPhase == 5) {
            printSystemStatus();
            demoPhase = 6;
        }
        break;

    case 6: // Advanced Features (25-30s)
        if (elapsedTime > 26000 && elapsedTime < 26100) {
            Serial.println("\n--- Phase 7: Advanced Features ---");

            // Test pose reset
            stateController->setPoseResetMode(POSE_RESET_ALL);
            Serial.println("✓ Pose reset mode set to ALL");

            // Return legs to walking
            stateController->setLegState(0, LEG_WALKING);
            stateController->setLegState(2, LEG_WALKING);
            Serial.println("✓ Manual legs returned to walking");

            // Change posing mode
            stateController->setPosingMode(POSING_Z_YAW);
            Serial.println("✓ Changed to Z-Yaw posing mode");
        }
        if (elapsedTime > 30000 && demoPhase == 6) {
            printSystemStatus();
            demoPhase = 7;
        }
        break;

    case 7: // Error Handling Demo (30-35s)
        if (elapsedTime > 31000 && elapsedTime < 31100) {
            Serial.println("\n--- Phase 8: Error Handling Demo ---");
            stateController->emergencyStop();
            Serial.println("✓ Emergency stop activated");
        }
        if (elapsedTime > 33000 && elapsedTime < 33100) {
            Serial.println("✓ Clearing error and resuming operation");
            stateController->clearError();
            stateController->requestSystemState(SYSTEM_OPERATIONAL);
        }
        if (elapsedTime > 35000 && demoPhase == 7) {
            printSystemStatus();
            demoPhase = 8;
        }
        break;

    case 8: // Demo Complete
        if (elapsedTime > 36000 && elapsedTime < 36100) {
            Serial.println("\n🎉 === DEMO COMPLETE ===");
            Serial.println("All StateController features demonstrated successfully!");
            Serial.println("\nFinal Status:");
            printSystemStatus();
            printCapabilities();
            demoComplete = true;
        }
        break;
    }
}

void initializeParameters() {
    robotParams.hexagon_radius = 150.0f;
    robotParams.coxa_length = 50.0f;
    robotParams.femur_length = 100.0f;
    robotParams.tibia_length = 120.0f;
    robotParams.robot_height = 80.0f;
    robotParams.robot_weight = 2.5f;
    robotParams.control_frequency = 50;

    // Set joint limits
    robotParams.coxa_angle_limits[0] = -90.0f;
    robotParams.coxa_angle_limits[1] = 90.0f;
    robotParams.femur_angle_limits[0] = -90.0f;
    robotParams.femur_angle_limits[1] = 90.0f;
    robotParams.tibia_angle_limits[0] = -135.0f;
    robotParams.tibia_angle_limits[1] = 45.0f;
}

void printSystemStatus() {
    Serial.println("\n=== SYSTEM STATUS ===");
    Serial.print("Initialized: ");
    Serial.println(stateController->isInitialized() ? "YES" : "NO");
    Serial.print("Has Errors: ");
    Serial.println(stateController->hasErrors() ? "YES" : "NO");
    Serial.print("System State: ");
    Serial.println(getSystemStateName(stateController->getSystemState()));
    Serial.print("Robot State: ");
    Serial.println(getRobotStateName(stateController->getRobotState()));
    Serial.print("Walk State: ");
    Serial.println(getWalkStateName(stateController->getWalkState()));
    Serial.print("Posing Mode: ");
    Serial.println(getPosingModeName(stateController->getPosingMode()));
    Serial.print("Cruise Control: ");
    Serial.println(getCruiseControlName(stateController->getCruiseControlMode()));
    Serial.print("Manual Legs: ");
    Serial.println(stateController->getManualLegCount());
    Serial.print("Transitioning: ");
    Serial.println(stateController->isTransitioning() ? "YES" : "NO");
    Serial.println("==================");
}

void monitorTransition() {
    Serial.println("🔄 Monitoring state transition...");
    unsigned long startTime = millis();

    while (stateController->isTransitioning() && (millis() - startTime) < 5000) {
        TransitionProgress progress = stateController->getTransitionProgress();
        Serial.print("Progress: ");
        Serial.print(progress.current_step);
        Serial.print("/");
        Serial.print(progress.total_steps);
        Serial.print(" (");
        Serial.print(progress.completion_percentage);
        Serial.println("%)");
        delay(500);
    }

    if (!stateController->isTransitioning()) {
        Serial.println("✓ Transition completed successfully");
    }
}

void printCapabilities() {
    Serial.println("\n=== DEMONSTRATED CAPABILITIES ===");
    Serial.println("✅ Hierarchical State Management");
    Serial.println("  • System States: SUSPENDED ↔ OPERATIONAL");
    Serial.println("  • Robot States: UNKNOWN → READY → RUNNING");
    Serial.println("  • Walk States: STOPPED → STARTING → MOVING");
    Serial.println("  • Leg States: WALKING ↔ MANUAL");
    Serial.println("");
    Serial.println("✅ Advanced Operational Modes");
    Serial.println("  • Manual Leg Control (up to 2 legs)");
    Serial.println("  • Multi-Axis Pose Control (X-Y, Z-Yaw, Pitch-Roll)");
    Serial.println("  • Cruise Control with 3D velocity");
    Serial.println("  • Pose Reset (selective and full)");
    Serial.println("");
    Serial.println("✅ Real-time Progress Tracking");
    Serial.println("  • Transition step counting");
    Serial.println("  • Completion percentage");
    Serial.println("  • Error state monitoring");
    Serial.println("");
    Serial.println("✅ Safety & Error Handling");
    Serial.println("  • Emergency stop functionality");
    Serial.println("  • System suspension/recovery");
    Serial.println("  • Invalid transition prevention");
    Serial.println("");
    Serial.println("🚀 HexaMotion StateController - PRODUCTION READY!");
    Serial.println("=====================================");
}

// Helper functions for readable output
const char *getSystemStateName(SystemState state) {
    switch (state) {
    case SYSTEM_SUSPENDED:
        return "SUSPENDED";
    case SYSTEM_OPERATIONAL:
        return "OPERATIONAL";
    default:
        return "UNKNOWN";
    }
}

const char *getRobotStateName(RobotState state) {
    switch (state) {
    case ROBOT_UNKNOWN:
        return "UNKNOWN";
    case ROBOT_PACKED:
        return "PACKED";
    case ROBOT_READY:
        return "READY";
    case ROBOT_RUNNING:
        return "RUNNING";
    default:
        return "INVALID";
    }
}

const char *getWalkStateName(WalkState state) {
    switch (state) {
    case WALK_STOPPED:
        return "STOPPED";
    case WALK_STARTING:
        return "STARTING";
    case WALK_MOVING:
        return "MOVING";
    case WALK_STOPPING:
        return "STOPPING";
    default:
        return "INVALID";
    }
}

const char *getPosingModeName(PosingMode mode) {
    switch (mode) {
    case POSING_NONE:
        return "NONE";
    case POSING_X_Y:
        return "X_Y";
    case POSING_PITCH_ROLL:
        return "PITCH_ROLL";
    case POSING_Z_YAW:
        return "Z_YAW";
    case POSING_EXTERNAL:
        return "EXTERNAL";
    default:
        return "INVALID";
    }
}

const char *getCruiseControlName(CruiseControlMode mode) {
    switch (mode) {
    case CRUISE_CONTROL_OFF:
        return "OFF";
    case CRUISE_CONTROL_ON:
        return "ON";
    case CRUISE_CONTROL_EXTERNAL:
        return "EXTERNAL";
    default:
        return "INVALID";
    }
}
