/**
 * @file fsr_threshold_configuration_example.ino
 * @brief Example demonstrating FSR threshold configuration in HexaMotion
 *
 * This example shows how to configure FSR touchdown and liftoff thresholds
 * for optimal terrain detection based on your specific robot and sensors.
 */

#include "HexaMotion.h"

// Mock interfaces for demonstration
class MockServoInterface : public IServoInterface {
  public:
    bool initialize() override { return true; }
    bool hasBlockingStatusFlags(int, int) override { return false; }
    bool setJointAngleAndSpeed(int, int, float, float) override { return true; }
    float getJointAngle(int, int) override { return 0.0f; }
    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }
    bool setSpeed(int, int, float) override { return true; }
};

class MockFSRInterface : public IFSRInterface {
  private:
    float readings_[NUM_LEGS] = {0};

  public:
    bool initialize() override { return true; }
    bool calibrateFSR(int) override { return true; }
    float getRawReading(int leg) override {
        return readings_[leg];
    }

    FSRData getFSRData(int leg) override {
        return {readings_[leg], readings_[leg] > 0, readings_[leg]};
    }

    // Helper method to simulate FSR readings
    void setReading(int leg, float pressure) {
        if (leg >= 0 && leg < NUM_LEGS) {
            readings_[leg] = pressure;
        }
    }
};

class MockIMUInterface : public IIMUInterface {
  public:
    bool initialize() override { return true; }
    bool calibrateIMU() override { return true; }

    IMUData readIMU() override {
        IMUData data;
        data.is_valid = true;
        data.has_absolute_capability = false;
        data.acceleration = {0, 0, -9.81f};
        data.angular_velocity = {0, 0, 0};
        data.orientation = {0, 0, 0, 1}; // Quaternion (w, x, y, z)
        return data;
    }
};

// Global interfaces
MockServoInterface servo_interface;
MockFSRInterface fsr_interface;
MockIMUInterface imu_interface;
LocomotionSystem *locomotion_system;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("FSR Threshold Configuration Example");
    Serial.println("===================================");

    // Configure robot parameters
    Parameters robot_params = createBasicParameters();

    // Method 1: Configure FSR thresholds in robot parameters
    Serial.println("\n=== Method 1: Configuration via Parameters ===");

    // Configure touchdown threshold (when foot makes contact)
    robot_params.fsr_touchdown_threshold = 30.0f; // 30 ADC units for touchdown

    // Configure liftoff threshold (when foot loses contact)
    robot_params.fsr_liftoff_threshold = 15.0f; // 15 ADC units for liftoff

    Serial.print("Configured touchdown threshold: ");
    Serial.println(robot_params.fsr_touchdown_threshold);
    Serial.print("Configured liftoff threshold: ");
    Serial.println(robot_params.fsr_liftoff_threshold);

    // Create locomotion system with configured thresholds
    locomotion_system = new LocomotionSystem(robot_params);

    // Initialize with interfaces
    if (locomotion_system->initialize(&servo_interface, &fsr_interface, &imu_interface)) {
        Serial.println("✓ Locomotion system initialized successfully");
    } else {
        Serial.println("✗ Failed to initialize locomotion system");
        return;
    }

    // Method 2: Dynamic threshold configuration
    Serial.println("\n=== Method 2: Dynamic Configuration ===");

    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    // Check current thresholds
    Serial.print("Current touchdown threshold: ");
    Serial.println(terrain_adaptation.getTouchdownThreshold());
    Serial.print("Current liftoff threshold: ");
    Serial.println(terrain_adaptation.getLiftoffThreshold());

    // Update thresholds dynamically
    terrain_adaptation.setTouchdownThreshold(40.0f);
    terrain_adaptation.setLiftoffThreshold(20.0f);

    Serial.print("Updated touchdown threshold: ");
    Serial.println(terrain_adaptation.getTouchdownThreshold());
    Serial.print("Updated liftoff threshold: ");
    Serial.println(terrain_adaptation.getLiftoffThreshold());

    // Method 3: Terrain-adaptive thresholds
    Serial.println("\n=== Method 3: Terrain-Adaptive Configuration ===");
    demonstrateTerrainAdaptiveThresholds();

    // Method 4: Calibration-based thresholds
    Serial.println("\n=== Method 4: Calibration-Based Configuration ===");
    demonstrateCalibrationBasedThresholds();

    Serial.println("\n=== Configuration Complete ===");
    Serial.println("The robot is now ready for terrain-aware locomotion!");
}

void demonstrateTerrainAdaptiveThresholds() {
    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    // Simulate different terrain types
    struct TerrainConfig {
        const char *name;
        float touchdown_threshold;
        float liftoff_threshold;
        const char *description;
    };

    TerrainConfig terrains[] = {
        {"Hard surfaces", 50.0f, 25.0f, "Concrete, tiles - higher thresholds"},
        {"Soft surfaces", 20.0f, 10.0f, "Carpet, grass - lower thresholds"},
        {"Sand/gravel", 35.0f, 15.0f, "Loose materials - medium thresholds"},
        {"Uneven terrain", 45.0f, 20.0f, "Rocks, obstacles - higher sensitivity"}};

    for (auto &terrain : terrains) {
        Serial.print("Configuring for ");
        Serial.print(terrain.name);
        Serial.print(": ");
        Serial.println(terrain.description);

        terrain_adaptation.setTouchdownThreshold(terrain.touchdown_threshold);
        terrain_adaptation.setLiftoffThreshold(terrain.liftoff_threshold);

        Serial.print("  Touchdown: ");
        Serial.print(terrain.touchdown_threshold);
        Serial.print(", Liftoff: ");
        Serial.println(terrain.liftoff_threshold);

        delay(500); // Simulate terrain change
    }
}

void demonstrateCalibrationBasedThresholds() {
    auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();

    Serial.println("Performing FSR calibration...");

    // Simulate reading unloaded FSR values (noise floor)
    float noise_floor = 5.0f;
    Serial.print("Measured noise floor: ");
    Serial.println(noise_floor);

    // Simulate reading loaded FSR values (robot weight)
    float weight_reading = 80.0f;
    Serial.print("Measured weight reading: ");
    Serial.println(weight_reading);

    // Calculate optimal thresholds based on calibration
    float touchdown_threshold = noise_floor + (weight_reading - noise_floor) * 0.3f; // 30% above noise
    float liftoff_threshold = noise_floor + (weight_reading - noise_floor) * 0.1f;   // 10% above noise

    Serial.print("Calculated touchdown threshold: ");
    Serial.println(touchdown_threshold);
    Serial.print("Calculated liftoff threshold: ");
    Serial.println(liftoff_threshold);

    // Apply calibrated thresholds
    terrain_adaptation.setTouchdownThreshold(touchdown_threshold);
    terrain_adaptation.setLiftoffThreshold(liftoff_threshold);

    Serial.println("✓ Calibration-based thresholds applied");
}

void loop() {
    // Demonstrate FSR threshold usage during operation
    static unsigned long last_demo = 0;
    static int demo_state = 0;

    if (millis() - last_demo > 3000) { // Every 3 seconds
        last_demo = millis();

        Serial.println("\n=== Simulating Ground Contact Events ===");

        auto &terrain_adaptation = locomotion_system->getTerrainAdaptation();
        float touchdown_threshold = terrain_adaptation.getTouchdownThreshold();
        float liftoff_threshold = terrain_adaptation.getLiftoffThreshold();

        switch (demo_state) {
        case 0: {
            // Simulate leg 0 touching down
            float pressure = touchdown_threshold + 5.0f; // Above threshold
            fsr_interface.setReading(0, pressure);
            Serial.print("Leg 0 touchdown detected (pressure: ");
            Serial.print(pressure);
            Serial.print(" > threshold: ");
            Serial.print(touchdown_threshold);
            Serial.println(")");
            break;
        }

        case 1: {
            // Simulate leg 0 lifting off
            float pressure = liftoff_threshold - 2.0f; // Below threshold
            fsr_interface.setReading(0, pressure);
            Serial.print("Leg 0 liftoff detected (pressure: ");
            Serial.print(pressure);
            Serial.print(" < threshold: ");
            Serial.print(liftoff_threshold);
            Serial.println(")");
            break;
        }

        case 2: {
            // Demonstrate hysteresis (pressure between thresholds)
            float pressure = (touchdown_threshold + liftoff_threshold) / 2.0f;
            fsr_interface.setReading(0, pressure);
            Serial.print("Pressure in hysteresis zone (");
            Serial.print(pressure);
            Serial.print(" between ");
            Serial.print(liftoff_threshold);
            Serial.print(" and ");
            Serial.print(touchdown_threshold);
            Serial.println(") - no state change");
            break;
        }
        }

        // Update locomotion system to process FSR data
        locomotion_system->update();

        // Check touchdown detection status
        bool has_touchdown = terrain_adaptation.hasTouchdownDetection(0);
        Serial.print("Leg 0 touchdown status: ");
        Serial.println(has_touchdown ? "CONTACT" : "NO CONTACT");

        demo_state = (demo_state + 1) % 3;
    }

    delay(100);
}

/**
 * @brief Create basic robot parameters for demonstration
 * @return Default robot parameters
 */
Parameters createBasicParameters() {
    Parameters params{};

    // Robot geometry (example values)
    params.hexagon_radius = 400.0f; // mm
    params.coxa_length = 50.0f;     // mm
    params.femur_length = 101.0f;   // mm
    params.tibia_length = 208.0f;   // mm
    params.robot_height = 90.0f;    // mm

    // Joint limits
    params.coxa_angle_limits[0] = -65.0f;  // degrees
    params.coxa_angle_limits[1] = 65.0f;   // degrees
    params.femur_angle_limits[0] = -75.0f; // degrees
    params.femur_angle_limits[1] = 75.0f;  // degrees
    params.tibia_angle_limits[0] = -45.0f; // degrees
    params.tibia_angle_limits[1] = 45.0f;  // degrees

    // Control parameters
    params.max_velocity = 100.0f;        // mm/s
    params.max_angular_velocity = 45.0f; // degrees/s
    params.control_frequency = 50.0f;    // Hz

    // FSR parameters (will be configured in example)
    params.fsr_touchdown_threshold = 0.0f; // Will be set dynamically
    params.fsr_liftoff_threshold = 0.0f;   // Will be set dynamically
    params.fsr_max_pressure = 100.0f;      // Maximum FSR reading (N)

    return params;
}
