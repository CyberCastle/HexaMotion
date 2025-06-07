#ifndef MODEL_H
#define MODEL_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include "math_utils.h"

// System configuration
#define NUM_LEGS 6
#define DOF_PER_LEG 3
#define TOTAL_DOF (NUM_LEGS * DOF_PER_LEG)

// Configurable physical parameters
struct Parameters {
    float hexagon_radius;
    float coxa_length;
    float femur_length;
    float tibia_length;

    float robot_height;
    float robot_weight;
    Eigen::Vector3f center_of_mass;

    float coxa_angle_limits[2];
    float femur_angle_limits[2];
    float tibia_angle_limits[2];

    float dh_parameters[NUM_LEGS][DOF_PER_LEG][4];

    Eigen::Vector3f imu_calibration_offset;
    float fsr_threshold;
    float fsr_max_pressure;

    float max_velocity;
    float max_angular_velocity;
    float stability_margin;
    float control_frequency;

    struct IKConfig {
        uint8_t max_iterations = 30;
        float pos_threshold_mm = 0.5f;
        bool use_damping = true;
        float damping_lambda = 30.0f;
        bool clamp_joints = true;
    } ik;

    struct BodyCompConfig {
        bool enable = true;
        float kp = 0.6f;
        float lp_alpha = 0.10f;
        float max_tilt_deg = 12.0f;
    } body_comp;
};

enum GaitType {
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

enum LegState {
    STANCE_PHASE,
    SWING_PHASE,
    LIFT_PHASE,
    TOUCHDOWN_PHASE
};

struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

struct JointAngles {
    float coxa, femur, tibia;
    JointAngles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

struct IMUData {
    float roll, pitch, yaw;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    bool is_valid;
};

struct FSRData {
    float pressure;
    bool in_contact;
    float contact_time;
};

class IIMUInterface {
  public:
    virtual ~IIMUInterface() = default;
    virtual bool initialize() = 0;
    virtual IMUData readIMU() = 0;
    virtual bool calibrate() = 0;
    virtual bool isConnected() = 0;
};

class IFSRInterface {
  public:
    virtual ~IFSRInterface() = default;
    virtual bool initialize() = 0;
    virtual FSRData readFSR(int leg_index) = 0;
    virtual bool calibrateFSR(int leg_index) = 0;
    virtual float getRawReading(int leg_index) = 0;
};

class IServoInterface {
  public:
    virtual ~IServoInterface() = default;
    virtual bool initialize() = 0;
    virtual bool setJointAngle(int leg_index, int joint_index, float angle) = 0;
    virtual float getJointAngle(int leg_index, int joint_index) = 0;
    virtual bool setJointSpeed(int leg_index, int joint_index, float speed) = 0;
    virtual bool isJointMoving(int leg_index, int joint_index) = 0;
    virtual bool enableTorque(int leg_index, int joint_index, bool enable) = 0;
};

class RobotModel {
  public:
    explicit RobotModel(const Parameters &params);

    JointAngles inverseKinematics(int leg, const Point3D &p);
    Point3D forwardKinematics(int leg, const JointAngles &q);
    Eigen::Matrix3f analyticJacobian(int leg, const JointAngles &q);
    Eigen::Matrix4f legTransform(int leg, const JointAngles &q);
    bool checkJointLimits(int leg, const JointAngles &q) const;
    float constrainAngle(float angle, float min_angle, float max_angle) const;
    bool validate() const;
    const Parameters &getParams() const { return params; }

  private:
    const Parameters &params;
};

#endif // MODEL_H
