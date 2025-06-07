#ifndef HEXAMOTION_MOCK_INTERFACES_H
#define HEXAMOTION_MOCK_INTERFACES_H

#include <Arduino.h>
#include <hexapod_locomotion_system.h>

/**
 * @brief Example IMU interface returning mock data.
 */
class ExampleIMU : public IIMUInterface {
  public:
    bool initialize() override { return true; }
    IMUData readIMU() override {
        IMUData data{};
        data.roll = 0.0f;
        data.pitch = 0.0f;
        data.yaw = 0.0f;
        data.accel_x = 0.0f;
        data.accel_y = 0.0f;
        data.accel_z = 9.8f;
        data.gyro_x = 0.0f;
        data.gyro_y = 0.0f;
        data.gyro_z = 0.0f;
        data.is_valid = true;
        return data;
    }
    bool calibrate() override { return true; }
    bool isConnected() override { return true; }
};

/**
 * @brief Example FSR interface providing constant readings.
 */
class ExampleFSR : public IFSRInterface {
  public:
    bool initialize() override { return true; }
    FSRData readFSR(int /*leg_index*/) override {
        FSRData data{};
        data.pressure = 0.0f;
        data.in_contact = false;
        data.contact_time = 0.0f;
        return data;
    }
    bool calibrateFSR(int /*leg_index*/) override { return true; }
    float getRawReading(int /*leg_index*/) override { return 0.0f; }
};

/**
 * @brief Example servo interface storing commanded angles in RAM.
 */
class ExampleServo : public IServoInterface {
  public:
    ExampleServo() {
        for (int i = 0; i < NUM_LEGS; ++i) {
            for (int j = 0; j < DOF_PER_LEG; ++j) {
                angles[i][j] = 0.0f;
            }
        }
    }

    bool initialize() override { return true; }

    bool setJointAngle(int leg_index, int joint_index, float angle) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return false;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG) return false;
        angles[leg_index][joint_index] = angle;
        return true;
    }

    float getJointAngle(int leg_index, int joint_index) override {
        if (leg_index < 0 || leg_index >= NUM_LEGS) return 0.0f;
        if (joint_index < 0 || joint_index >= DOF_PER_LEG) return 0.0f;
        return angles[leg_index][joint_index];
    }

    bool setJointSpeed(int /*leg_index*/, int /*joint_index*/, float /*speed*/) override { return true; }
    bool isJointMoving(int /*leg_index*/, int /*joint_index*/) override { return false; }
    bool enableTorque(int /*leg_index*/, int /*joint_index*/, bool /*enable*/) override { return true; }

  private:
    float angles[NUM_LEGS][DOF_PER_LEG];
};

#endif // HEXAMOTION_MOCK_INTERFACES_H
