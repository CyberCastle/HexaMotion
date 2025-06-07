#ifndef TEST_STUBS_H
#define TEST_STUBS_H

#include <iostream>
#include "../include/model.h"

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

#endif // TEST_STUBS_H
