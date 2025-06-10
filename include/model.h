#ifndef MODEL_H
#define MODEL_H

#include "math_utils.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <utility>

// System configuration
#define NUM_LEGS 6
#define DOF_PER_LEG 3
#define TOTAL_DOF (NUM_LEGS * DOF_PER_LEG)

/**
 * @brief Robot configuration parameters.
 */
struct Parameters {
    float hexagon_radius;
    float coxa_length;
    float femur_length;
    float tibia_length;

    float robot_height;
    float height_offset = 0.0f; ///< structural body height offset
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

    /**
     * @brief Inverse kinematics solver settings.
     */
    struct IKConfig {
        uint8_t max_iterations = 30;
        float pos_threshold_mm = 0.5f;
        bool use_damping = true;
        float damping_lambda = 30.0f;
        bool clamp_joints = true;
        bool use_multiple_starts = true; ///< Enable multiple starting configurations for better convergence
    } ik;

    /**
     * @brief Body compensation filter parameters.
     */
    struct BodyCompConfig {
        bool enable = true;
        float kp = 0.6f;
        float lp_alpha = 0.10f;
        float max_tilt_deg = 12.0f;
    } body_comp;

    // Gait step calculation factors
    /**
     * @brief Factors for calculating gait step length and height.
     */
    struct GaitFactors {
        // Step length as percentage of leg reach
        float tripod_length_factor = 0.35f;      // 35% for speed
        float wave_length_factor = 0.25f;        // 25% for stability
        float ripple_length_factor = 0.30f;      // 30% for balance
        float metachronal_length_factor = 0.28f; // 28% for smooth motion
        float adaptive_length_factor = 0.30f;    // 30% base for adaptation

        // Step height as percentage of robot height
        float tripod_height_factor = 0.30f;      // 30% for obstacle clearance
        float wave_height_factor = 0.20f;        // 20% for stability
        float ripple_height_factor = 0.25f;      // 25% for balance
        float metachronal_height_factor = 0.25f; // 25% for smooth motion
        float adaptive_height_factor = 0.25f;    // 25% base for adaptation

        // Safety limits as percentage of leg reach/height
        float min_length_factor = 0.12f; // 12% minimum step length
        float max_length_factor = 0.50f; // 50% maximum step length
        float min_height_factor = 0.12f; // 12% minimum step height
        float max_height_factor = 0.40f; // 40% maximum step height
    } gait_factors;
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

/**
 * @brief 3D coordinate in millimeters.
 */
struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

/**
 * @brief Joint angles for a single leg in degrees.
 */
struct JointAngles {
    float coxa, femur, tibia;
    JointAngles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

/**
 * @brief Inertial measurement unit readings.
 */
struct IMUData {
    float roll, pitch, yaw;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    bool is_valid;
};

/**
 * @brief Force sensing resistor data for a foot.
 */
struct FSRData {
    float pressure;
    bool in_contact;
    float contact_time;
};

class IIMUInterface {
  public:
    virtual ~IIMUInterface() = default;
    /** Initialize the IMU hardware. */
    virtual bool initialize() = 0;
    /** Retrieve current IMU data. */
    virtual IMUData readIMU() = 0;
    /** Calibrate the IMU sensors. */
    virtual bool calibrate() = 0;
    /** Check if the IMU is connected. */
    virtual bool isConnected() = 0;
};

class IFSRInterface {
  public:
    virtual ~IFSRInterface() = default;
    /** Initialize FSR sensors. */
    virtual bool initialize() = 0;
    /** Read FSR data for a leg. */
    virtual FSRData readFSR(int leg_index) = 0;
    /** Calibrate a specific FSR sensor. */
    virtual bool calibrateFSR(int leg_index) = 0;
    /** Get raw ADC reading from a sensor. */
    virtual float getRawReading(int leg_index) = 0;
};

class IServoInterface {
  public:
    virtual ~IServoInterface() = default;
    /** Initialize servo communication. */
    virtual bool initialize() = 0;
    /** Set a joint angle in degrees. */
    virtual bool setJointAngle(int leg_index, int joint_index, float angle) = 0;
    /** Retrieve the current joint angle. */
    virtual float getJointAngle(int leg_index, int joint_index) = 0;
    /** Set the servo speed. */
    virtual bool setJointSpeed(int leg_index, int joint_index, float speed) = 0;
    /** Check if a joint is currently moving. */
    virtual bool isJointMoving(int leg_index, int joint_index) = 0;
    /** Enable or disable torque on a joint. */
    virtual bool enableTorque(int leg_index, int joint_index, bool enable) = 0;
};

class RobotModel {
  public:
    /**
     * @brief Construct a robot model using the provided parameters.
     * @param params Reference to configuration parameters.
     */
    explicit RobotModel(const Parameters &params);

    /**
     * \brief Initialize DH parameters from robot dimensions.
     */
    void initializeDH();

    /** Compute inverse kinematics for a leg. */
    JointAngles inverseKinematics(int leg, const Point3D &p);
    /** Compute forward kinematics for a leg. */
    Point3D forwardKinematics(int leg, const JointAngles &q) const;
    /** Analytic Jacobian for a leg. */
    Eigen::Matrix3f analyticJacobian(int leg, const JointAngles &q) const;
    /** Numerical Jacobian calculation. */
    Eigen::Matrix3f calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const;
    /** Homogeneous transform for a full leg chain. */
    Eigen::Matrix4f legTransform(int leg, const JointAngles &q) const;
    /** Verify if joint angles are within defined limits. */
    bool checkJointLimits(int leg, const JointAngles &q) const;
    /** Clamp angle within limits. */
    float constrainAngle(float angle, float min_angle, float max_angle) const;
    /** Normalize angle to [-180,180] degrees. */
    float normalizeAngle(float angle_deg) const;
    /** Validate parameter consistency. */
    bool validate() const;
    /**
     * \brief Calculate minimal and maximal body height based on joint limits.
     * \return Pair {min_height, max_height} in millimeters.
     */
    std::pair<float, float> calculateHeightRange() const;
    const Parameters &getParams() const { return params; }

  private:
    const Parameters &params;
    float dh[NUM_LEGS][DOF_PER_LEG][4];
};

#endif // MODEL_H
