#ifndef TEST_STUBS_H
#define TEST_STUBS_H

#include "../include/model.h"
#include <iostream>

struct DummyIMU : IIMUInterface {
    bool initialize() override { return true; }
    IMUData readIMU() override {
        IMUData data{};
        data.roll = data.pitch = data.yaw = 0.f;
        data.is_valid = true;
        return data;
    }
    bool calibrate() override { return true; }
    bool isConnected() override { return true; }
};

struct DummyFSR : IFSRInterface {
    bool initialize() override { return true; }
    FSRData readFSR(int) override { return FSRData{0.0f, true, 0.0f}; }
    bool calibrateFSR(int) override { return true; }
    float getRawReading(int) override { return 0.0f; }
};

struct DummyServo : IServoInterface {
    bool initialize() override { return true; }
    bool setJointAngle(int, int, float) override { return true; }
    float getJointAngle(int, int) override { return 0.0f; }
    bool setJointSpeed(int, int, float) override { return true; }
    bool isJointMoving(int, int) override { return false; }
    bool enableTorque(int, int, bool) override { return true; }
};

inline Parameters createDefaultParameters() {
    Parameters params;
    params.hexagon_radius = 90.0f;
    params.coxa_length = 52.0f;
    params.femur_length = 66.0f;
    params.tibia_length = 133.0f;
    params.robot_height = 80.0f;
    params.robot_weight = 2.0f;
    params.center_of_mass = Eigen::Vector3f(0, 0, 0);
    params.coxa_angle_limits[0] = -45.0f;
    params.coxa_angle_limits[1] = 45.0f;
    params.femur_angle_limits[0] = -90.0f;
    params.femur_angle_limits[1] = 90.0f;
    params.tibia_angle_limits[0] = -135.0f;
    params.tibia_angle_limits[1] = 45.0f;
    params.max_velocity = 100.0f;
    params.max_angular_velocity = 45.0f;
    params.stability_margin = 0.02f;
    params.control_frequency = 50.0f;
    params.fsr_threshold = 0.1f;
    params.fsr_max_pressure = 10.0f;
    return params;
}

#endif // TEST_STUBS_H
