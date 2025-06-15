/**
 * @file cruise_control_example.ino
 * @brief Example demonstrating the enhanced cruise control functionality
 *
 * This example shows how to use the enhanced cruise control features that are
 * now equivalent to OpenSHC's implementation.
 */

#include "locomotion_system.h"
#include "state_controller.h"

// Create locomotion system and state controller
LocomotionSystem locomotion_system;
StateMachineConfig config;
StateController state_controller(locomotion_system, config);

void setup() {
    Serial.begin(115200);

    // Configure cruise control settings
    config.enable_cruise_control = true;
    config.cruise_control_time_limit = 30.0f; // 30 second time limit

    // Initialize the state controller
    state_controller.initialize();

    Serial.println("Enhanced Cruise Control Example");
    Serial.println("===============================");
}

void loop() {
    // Example 1: Set cruise control with specific velocity
    Serial.println("\n1. Setting cruise control with specific velocity");
    Eigen::Vector3f cruise_vel(100.0f, 50.0f, 15.0f); // x, y, angular_z

    if (state_controller.setCruiseControlMode(CruiseControlMode::CRUISE_CONTROL_ON, cruise_vel)) {
        Serial.println("Cruise control enabled with velocity: [100, 50, 15]");
        Serial.print("Remaining time: ");
        Serial.print(state_controller.getCruiseRemainingTime());
        Serial.println(" seconds");
    }

    delay(2000);

    // Example 2: Set cruise control using current velocity
    Serial.println("\n2. Setting cruise control with current velocity");
    state_controller.setDesiredVelocity(Eigen::Vector2f(80.0f, -30.0f), 10.0f);

    if (state_controller.setCruiseControlMode(CruiseControlMode::CRUISE_CONTROL_ON)) {
        Serial.println("Cruise control enabled with current velocity");
        Eigen::Vector3f current_cruise = state_controller.getCruiseVelocity();
        Serial.print("Cruise velocity: [");
        Serial.print(current_cruise.x());
        Serial.print(", ");
        Serial.print(current_cruise.y());
        Serial.print(", ");
        Serial.print(current_cruise.z());
        Serial.println("]");
    }

    delay(5000);

    // Example 3: Monitor cruise control status
    Serial.println("\n3. Monitoring cruise control status");
    while (state_controller.isCruiseControlActive()) {
        Serial.print("Cruise active - Remaining time: ");
        Serial.print(state_controller.getCruiseRemainingTime());
        Serial.println(" seconds");

        // Update the state controller (this would normally be called in main loop)
        state_controller.update();

        delay(1000);

        // Break after a few iterations for demo
        static int counter = 0;
        if (++counter > 10)
            break;
    }

    // Example 4: Disable cruise control
    Serial.println("\n4. Disabling cruise control");
    state_controller.setCruiseControlMode(CruiseControlMode::CRUISE_CONTROL_OFF);
    Serial.println("Cruise control disabled");

    delay(5000);
}

/**
 * @brief Example function showing advanced cruise control usage
 */
void advancedCruiseControlExample() {
    // Set unlimited cruise control
    StateMachineConfig unlimited_config;
    unlimited_config.cruise_control_time_limit = 0.0f; // Unlimited

    StateController unlimited_controller(locomotion_system, unlimited_config);

    // This will run indefinitely until manually stopped
    Eigen::Vector3f forward_velocity(150.0f, 0.0f, 0.0f);
    unlimited_controller.setCruiseControlMode(CruiseControlMode::CRUISE_CONTROL_ON, forward_velocity);

    Serial.println("Unlimited cruise control enabled");

    // The robot will continue moving forward at 150 mm/s until:
    // 1. Manually disabled
    // 2. An error occurs
    // 3. System state changes
}
