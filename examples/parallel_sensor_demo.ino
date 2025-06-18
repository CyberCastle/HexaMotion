/**
 * @file parallel_sensor_demo.ino
 * @brief Demonstration of parallel FSR and IMU sensor reading capabilities
 *
 * This example demonstrates the new parallel sensor reading functionality
 * implemented in HexaMotion, showing the performance benefits and timing
 * improvements achieved by simultaneous FSR and IMU data acquisition.
 *
 * Features demonstrated:
 * - Parallel FSR + IMU sensor updates
 * - Performance monitoring and timing analysis
 * - Synchronized sensor data acquisition
 * - Error handling and fallback mechanisms
 */

#include <Arduino.h>
#include <locomotion_system.h>

// Performance monitoring structure
struct SensorPerformanceMonitor {
    unsigned long parallel_update_time;
    unsigned long sequential_update_time;
    unsigned long last_measurement_time;
    uint32_t update_count;
    float avg_parallel_time;
    float avg_sequential_time;
    bool parallel_mode_enabled;

    SensorPerformanceMonitor() : parallel_update_time(0), sequential_update_time(0),
                                 last_measurement_time(0), update_count(0),
                                 avg_parallel_time(0), avg_sequential_time(0),
                                 parallel_mode_enabled(true) {}

    void reset() {
        update_count = 0;
        avg_parallel_time = 0;
        avg_sequential_time = 0;
    }
};

// Enhanced IMU mock with timing simulation
class ParallelDemoIMU : public IIMUInterface {
  private:
    IMUMode current_mode_;
    bool supports_absolute_;
    float simulated_roll_, simulated_pitch_, simulated_yaw_;
    unsigned long last_update_time_;
    bool data_ready_;

  public:
    ParallelDemoIMU() : current_mode_(IMU_MODE_RAW_DATA), supports_absolute_(false),
                        simulated_roll_(0), simulated_pitch_(0), simulated_yaw_(0),
                        last_update_time_(0), data_ready_(false) {}

    bool initialize() override {
        Serial.println("ParallelDemo IMU: Initializing...");
        return true;
    }

    bool isConnected() override { return true; }
    bool calibrate() override { return true; }

    bool setIMUMode(IMUMode mode) override {
        current_mode_ = mode;
        return true;
    }

    IMUMode getIMUMode() const override { return current_mode_; }
    bool hasAbsolutePositioning() const override { return supports_absolute_; }

    bool getCalibrationStatus(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) override {
        if (system)
            *system = 3;
        if (gyro)
            *gyro = 3;
        if (accel)
            *accel = 3;
        if (mag)
            *mag = 3;
        return true;
    }

    bool runSelfTest() override { return true; }
    bool resetOrientation() override { return true; }

    bool update() override {
        // Simulate parallel sensor update with realistic timing
        unsigned long start_time = micros();

        // Simulate IMU data acquisition (non-blocking)
        simulated_roll_ += 0.1f * sin(millis() * 0.001f);
        simulated_pitch_ += 0.05f * cos(millis() * 0.002f);
        simulated_yaw_ += 0.02f;

        // Simulate processing time for real IMU
        delayMicroseconds(800); // Typical BNO055 I2C read time

        last_update_time_ = micros() - start_time;
        data_ready_ = true;
        return true;
    }

    IMUData readIMU() override {
        IMUData data{};

        // Return cached data from last update() call
        data.roll = simulated_roll_;
        data.pitch = simulated_pitch_;
        data.yaw = simulated_yaw_;
        data.accel_x = 0.1f * sin(millis() * 0.003f);
        data.accel_y = 0.1f * cos(millis() * 0.003f);
        data.accel_z = 9.81f + 0.05f * sin(millis() * 0.005f);
        data.gyro_x = 0.01f * sin(millis() * 0.004f);
        data.gyro_y = 0.01f * cos(millis() * 0.004f);
        data.gyro_z = 0.005f * sin(millis() * 0.002f);
        data.is_valid = data_ready_;
        data.mode = current_mode_;
        data.has_absolute_capability = supports_absolute_;

        return data;
    }

    unsigned long getLastUpdateTime() const { return last_update_time_; }
    void enableAbsoluteMode(bool enable) { supports_absolute_ = enable; }
};

// Enhanced FSR mock with DMA simulation
class ParallelDemoFSR : public IFSRInterface {
  private:
    FSRData sensor_data_[NUM_LEGS];
    unsigned long last_update_time_;
    bool dma_ready_;

  public:
    ParallelDemoFSR() : last_update_time_(0), dma_ready_(false) {
        for (int i = 0; i < NUM_LEGS; i++) {
            sensor_data_[i] = {0.0f, false, 0.0f};
        }
    }

    bool initialize() override {
        Serial.println("ParallelDemo FSR: Initializing AdvancedAnalog DMA...");
        return true;
    }

    bool calibrateFSR(int leg_index) override { return true; }

    float getRawReading(int leg_index) override {
        if (leg_index >= 0 && leg_index < NUM_LEGS) {
            return sensor_data_[leg_index].pressure;
        }
        return 0.0f;
    }

    bool update() override {
        // Simulate AdvancedAnalog DMA simultaneous read
        unsigned long start_time = micros();

        // Simulate simultaneous ADC reading of all 6 FSR channels
        for (int i = 0; i < NUM_LEGS; i++) {
            // Simulate contact patterns
            float time_offset = (millis() + i * 1000) * 0.001f;
            sensor_data_[i].pressure = 5.0f + 2.0f * sin(time_offset);
            sensor_data_[i].in_contact = sensor_data_[i].pressure > 4.0f;
            sensor_data_[i].contact_time = sensor_data_[i].in_contact ? 0.1f : 0.0f;
        }

        // Simulate DMA processing time
        delayMicroseconds(500); // Typical AdvancedAnalog DMA time for 6 channels

        last_update_time_ = micros() - start_time;
        dma_ready_ = true;
        return true;
    }

    FSRData readFSR(int leg_index) override {
        if (leg_index >= 0 && leg_index < NUM_LEGS) {
            return sensor_data_[leg_index];
        }
        return FSRData{0.0f, false, 0.0f};
    }

    unsigned long getLastUpdateTime() const { return last_update_time_; }
};

// Simple servo mock
class ParallelDemoServo : public IServoInterface {
  public:
    bool initialize() override { return true; }
    bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) override {
        (void)leg_index;
        (void)joint_index;
        (void)angle;
        (void)speed;
        return true;
    }
    float getJointAngle(int leg_index, int joint_index) override { return 0.0f; }
    bool isJointMoving(int leg_index, int joint_index) override { return false; }
    bool enableTorque(int leg_index, int joint_index, bool enable) override { return true; }
};

// Global objects
ParallelDemoIMU demo_imu;
ParallelDemoFSR demo_fsr;
ParallelDemoServo demo_servo;
LocomotionSystem locomotion_system;
SensorPerformanceMonitor perf_monitor;

void setup() {
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    Serial.println("=== HexaMotion Parallel Sensor Reading Demo ===");
    Serial.println("Demonstrating FSR + IMU parallel sensor acquisition");
    Serial.println();

    // Initialize locomotion system with demo interfaces
    Parameters params;
    params.coxa_length = 50.0f;
    params.femur_length = 80.0f;
    params.tibia_length = 120.0f;
    params.robot_height = 100.0f;

    locomotion_system = LocomotionSystem(params);

    if (!locomotion_system.initialize(&demo_imu, &demo_fsr, &demo_servo)) {
        Serial.println("ERROR: Failed to initialize locomotion system");
        return;
    }

    // Enable system
    locomotion_system.enable();

    Serial.println("System initialized successfully!");
    Serial.println("Starting parallel sensor reading demonstration...");
    Serial.println();

    // Print header for performance data
    Serial.println("Time(s) | Parallel(µs) | Sequential(µs) | Improvement | Errors");
    Serial.println("--------|--------------|----------------|-------------|-------");
}

void measureSequentialPerformance() {
    // Measure traditional sequential sensor reading
    unsigned long start_time = micros();

    // Sequential FSR reading
    demo_fsr.update();

    // Sequential IMU reading
    demo_imu.update();

    perf_monitor.sequential_update_time = micros() - start_time;
}

void printPerformanceReport() {
    static unsigned long last_report_time = 0;

    if (millis() - last_report_time < 2000)
        return; // Report every 2 seconds
    last_report_time = millis();

    // Update averages
    float parallel_time = perf_monitor.parallel_update_time;
    float sequential_time = perf_monitor.sequential_update_time;

    if (perf_monitor.update_count == 0) {
        perf_monitor.avg_parallel_time = parallel_time;
        perf_monitor.avg_sequential_time = sequential_time;
    } else {
        float alpha = 0.1f; // Low-pass filter
        perf_monitor.avg_parallel_time =
            (1.0f - alpha) * perf_monitor.avg_parallel_time + alpha * parallel_time;
        perf_monitor.avg_sequential_time =
            (1.0f - alpha) * perf_monitor.avg_sequential_time + alpha * sequential_time;
    }

    perf_monitor.update_count++;

    // Calculate improvement percentage
    float improvement = 0.0f;
    if (perf_monitor.avg_sequential_time > 0) {
        improvement = ((perf_monitor.avg_sequential_time - perf_monitor.avg_parallel_time) /
                       perf_monitor.avg_sequential_time) *
                      100.0f;
    }

    // Check for errors
    String error_status = "OK";
    if (locomotion_system.getLastError() != LocomotionSystem::NO_ERROR) {
        error_status = locomotion_system.getErrorMessage(locomotion_system.getLastError());
    }

    // Print performance data
    Serial.print(millis() / 1000.0f, 1);
    Serial.print("   |     ");
    Serial.print(perf_monitor.avg_parallel_time, 0);
    Serial.print("      |       ");
    Serial.print(perf_monitor.avg_sequential_time, 0);
    Serial.print("       |    ");
    Serial.print(improvement, 1);
    Serial.print("%     | ");
    Serial.println(error_status);
}

void demonstrateSensorData() {
    static unsigned long last_demo_time = 0;

    if (millis() - last_demo_time < 5000)
        return; // Demo every 5 seconds
    last_demo_time = millis();

    Serial.println();
    Serial.println("=== Current Sensor Data ===");

    // Show IMU data
    IMUData imu_data = demo_imu.readIMU();
    Serial.print("IMU - Roll: ");
    Serial.print(imu_data.roll, 2);
    Serial.print("°, Pitch: ");
    Serial.print(imu_data.pitch, 2);
    Serial.print("°, Yaw: ");
    Serial.print(imu_data.yaw, 2);
    Serial.println("°");

    // Show FSR data
    Serial.print("FSR - Contacts: ");
    for (int i = 0; i < NUM_LEGS; i++) {
        FSRData fsr_data = demo_fsr.readFSR(i);
        Serial.print(fsr_data.in_contact ? "1" : "0");
        if (i < NUM_LEGS - 1)
            Serial.print(",");
    }
    Serial.println();
    Serial.println("========================");
    Serial.println();
}

void loop() {
    // Measure parallel performance
    unsigned long parallel_start = micros();
    bool update_success = locomotion_system.update(); // Uses parallel sensor reading
    perf_monitor.parallel_update_time = micros() - parallel_start;

    // Measure sequential performance for comparison
    measureSequentialPerformance();

    // Performance reporting
    printPerformanceReport();

    // Sensor data demonstration
    demonstrateSensorData();

    // Handle any errors
    if (!update_success) {
        Serial.print("Update failed: ");
        Serial.println(locomotion_system.getErrorMessage(locomotion_system.getLastError()));
    }

    // Main loop rate: 50Hz (20ms period)
    delay(20);
}
