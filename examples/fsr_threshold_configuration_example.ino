/**
 * @file fsr_threshold_configuration_example.ino
 * @brief Example demonstrating FSR threshold configuration for terrain adaptation
 *
 * This example shows how to configure FSR thresholds for touchdown and liftoff detection
 * in the terrain adaptation system. It demonstrates both setting parameters during
 * initialization and updating them dynamically.
 */

#include "locomotion_system.h"
#include "mock_interfaces.h"

// Mock interfaces for demonstration
MockServoInterface servo_interface;
MockIMUInterface imu_interface;
MockFSRInterface fsr_interface;

LocomotionSystem *locomotion_system;

void setup() {
    Serial.begin(115200);
    Serial.println("FSR Threshold Configuration Example");

    // Create robot parameters
    Parameters robot_params = createBasicParameters();

    // ========================================================================
    // FSR THRESHOLD CONFIGURATION
    // ========================================================================

    // Configure FSR thresholds for terrain detection
    // These values should be calibrated based on your specific FSR sensors

    // Touchdown threshold - pressure level that indicates foot contact with ground
    // Higher values make the system less sensitive to light contact
    // Lower values make the system more sensitive but may trigger false positives
    robot_params.fsr_touchdown_threshold = 30.0f; // ADC units (touchdown)

    // Liftoff threshold - pressure level below which foot is considered lifted
    // Should be lower than touchdown threshold to provide hysteresis
    // This prevents oscillation between touchdown/liftoff states
    robot_params.fsr_liftoff_threshold = 15.0f; // ADC units (liftoff)

    // Maximum expected pressure for scaling (optional)
    robot_params.fsr_max_pressure = 200.0f; // ADC units

    // If thresholds are not set (left as 0.0f), the system will use defaults:
    // - DEFAULT_FSR_TOUCHDOWN_THRESHOLD (10.0f)
    // - DEFAULT_FSR_LIFTOFF_THRESHOLD (5.0f)

    // ========================================================================
    // INITIALIZE LOCOMOTION SYSTEM
    // ========================================================================

    locomotion_system = new LocomotionSystem(robot_params);
    locomotion_system->initialize(&servo_interface, &imu_interface, &fsr_interface);

    // ========================================================================
    // VERIFY THRESHOLD CONFIGURATION
    // ========================================================================

    // Get terrain adaptation system to verify configuration
    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    Serial.println("FSR Threshold Configuration:");
    Serial.print("Touchdown Threshold: ");
    Serial.println(terrain_adaptation.getTouchdownThreshold());
    Serial.print("Liftoff Threshold: ");
    Serial.println(terrain_adaptation.getLiftoffThreshold());

    // ========================================================================
    // DYNAMIC THRESHOLD UPDATES (Optional)
    // ========================================================================

    // You can also update thresholds dynamically during runtime
    // This is useful for adaptive systems or calibration routines

    // Example: Increase sensitivity for rough terrain
    if (digitalRead(2)) {                                // Assuming pin 2 is a "rough terrain" switch
        terrain_adaptation.setTouchdownThreshold(20.0f); // More sensitive
        terrain_adaptation.setLiftoffThreshold(10.0f);   // More sensitive
        Serial.println("Switched to rough terrain mode - increased sensitivity");
    }

    // Example: Decrease sensitivity for smooth surfaces
    if (digitalRead(3)) {                                // Assuming pin 3 is a "smooth surface" switch
        terrain_adaptation.setTouchdownThreshold(50.0f); // Less sensitive
        terrain_adaptation.setLiftoffThreshold(25.0f);   // Less sensitive
        Serial.println("Switched to smooth surface mode - decreased sensitivity");
    }

    Serial.println("Setup complete. Starting main loop...");
}

void loop() {
    // Update locomotion system
    locomotion_system->update();

    // Example: Monitor FSR readings and threshold performance
    static unsigned long last_report = 0;
    if (millis() - last_report > 1000) { // Report every second
        auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

        Serial.println("=== Terrain Detection Status ===");
        for (int leg = 0; leg < 6; leg++) {
            FSRData fsr_data = fsr_interface.readFSR(leg);
            bool has_contact = terrain_adaptation.hasTouchdownDetection(leg);

            Serial.print("Leg ");
            Serial.print(leg);
            Serial.print(": FSR=");
            Serial.print(fsr_data.pressure);
            Serial.print(", Contact=");
            Serial.println(has_contact ? "YES" : "NO");
        }

        Serial.print("Current Thresholds - Touchdown: ");
        Serial.print(terrain_adaptation.getTouchdownThreshold());
        Serial.print(", Liftoff: ");
        Serial.println(terrain_adaptation.getLiftoffThreshold());
        Serial.println();

        last_report = millis();
    }

    // Example: Automatic threshold adaptation based on average FSR readings
    static unsigned long last_adaptation = 0;
    if (millis() - last_adaptation > 5000) { // Adapt every 5 seconds
        adaptThresholdsBasedOnEnvironment();
        last_adaptation = millis();
    }

    delay(20); // 50Hz update rate
}

/**
 * @brief Example function for automatic threshold adaptation
 * This demonstrates how you might implement adaptive thresholds based on
 * environmental conditions or sensor calibration.
 */
void adaptThresholdsBasedOnEnvironment() {
    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    // Calculate average FSR reading across all legs
    float total_pressure = 0.0f;
    int active_legs = 0;

    for (int leg = 0; leg < 6; leg++) {
        FSRData fsr_data = fsr_interface.readFSR(leg);
        if (fsr_data.is_valid) {
            total_pressure += fsr_data.pressure;
            active_legs++;
        }
    }

    if (active_legs > 0) {
        float avg_pressure = total_pressure / active_legs;

        // Adaptive threshold based on average pressure
        // This is a simple example - real implementations would be more sophisticated
        float new_touchdown_threshold = avg_pressure * 0.6f; // 60% of average
        float new_liftoff_threshold = avg_pressure * 0.3f;   // 30% of average

        // Apply bounds to prevent unreasonable values
        new_touchdown_threshold = constrain(new_touchdown_threshold, 10.0f, 100.0f);
        new_liftoff_threshold = constrain(new_liftoff_threshold, 5.0f, 50.0f);

        // Update if significantly different
        if (abs(new_touchdown_threshold - terrain_adaptation.getTouchdownThreshold()) > 5.0f) {
            terrain_adaptation.setTouchdownThreshold(new_touchdown_threshold);
            terrain_adaptation.setLiftoffThreshold(new_liftoff_threshold);

            Serial.println("Adapted thresholds based on environment:");
            Serial.print("New Touchdown: ");
            Serial.println(new_touchdown_threshold);
            Serial.print("New Liftoff: ");
            Serial.println(new_liftoff_threshold);
        }
    }
}

/**
 * @brief Example calibration routine for FSR thresholds
 * This function demonstrates how to implement a calibration sequence
 * to determine optimal threshold values for a specific robot/environment.
 */
void calibrateFSRThresholds() {
    Serial.println("Starting FSR threshold calibration...");
    Serial.println("Please follow the instructions:");

    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    // Step 1: Measure baseline (robot lifted)
    Serial.println("1. Lift robot off ground and press any key...");
    while (!Serial.available())
        delay(100);
    Serial.read(); // Clear buffer

    float baseline_readings[6] = {0};
    for (int i = 0; i < 50; i++) { // Average 50 readings
        for (int leg = 0; leg < 6; leg++) {
            FSRData fsr_data = fsr_interface.readFSR(leg);
            baseline_readings[leg] += fsr_data.pressure;
        }
        delay(20);
    }

    for (int leg = 0; leg < 6; leg++) {
        baseline_readings[leg] /= 50.0f;
    }

    // Step 2: Measure contact (robot on ground)
    Serial.println("2. Place robot on ground and press any key...");
    while (!Serial.available())
        delay(100);
    Serial.read(); // Clear buffer

    float contact_readings[6] = {0};
    for (int i = 0; i < 50; i++) { // Average 50 readings
        for (int leg = 0; leg < 6; leg++) {
            FSRData fsr_data = fsr_interface.readFSR(leg);
            contact_readings[leg] += fsr_data.pressure;
        }
        delay(20);
    }

    for (int leg = 0; leg < 6; leg++) {
        contact_readings[leg] /= 50.0f;
    }

    // Calculate optimal thresholds
    float max_baseline = 0;
    float min_contact = 1000;

    for (int leg = 0; leg < 6; leg++) {
        max_baseline = max(max_baseline, baseline_readings[leg]);
        min_contact = min(min_contact, contact_readings[leg]);
    }

    // Set thresholds with safety margin
    float optimal_touchdown = max_baseline + (min_contact - max_baseline) * 0.3f;
    float optimal_liftoff = max_baseline + (min_contact - max_baseline) * 0.1f;

    terrain_adaptation.setTouchdownThreshold(optimal_touchdown);
    terrain_adaptation.setLiftoffThreshold(optimal_liftoff);

    Serial.println("Calibration complete!");
    Serial.print("Optimal Touchdown Threshold: ");
    Serial.println(optimal_touchdown);
    Serial.print("Optimal Liftoff Threshold: ");
    Serial.println(optimal_liftoff);
}
