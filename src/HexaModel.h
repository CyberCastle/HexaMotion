#ifndef MODEL_H
#define MODEL_H

#include "hexamotion_constants.h"
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
    float default_servo_speed = SERVO_SPEED_DEFAULT; ///< Default servo movement speed (0.1-3.0, where 1.0 is normal speed)

    /**
     * @brief Smooth trajectory configuration for pose updates.
     * Equivalent to OpenSHC's trajectory interpolation system.
     */
    struct SmoothTrajectoryConfig {
        bool use_current_servo_positions = true;          ///< Use current servo positions as starting point for trajectories (OpenSHC-style)
        bool enable_pose_interpolation = true;            ///< Enable smooth pose interpolation between positions
        float interpolation_speed = MIN_SERVO_VELOCITY;   ///< Interpolation speed factor (0.01-1.0, where 0.1 is smooth)
        float position_tolerance_mm = POSITION_TOLERANCE; ///< Position tolerance for determining if servo has reached target
        uint8_t max_interpolation_steps = 20;             ///< Maximum steps for pose interpolation
        bool use_quaternion_slerp = true;                 ///< Use spherical interpolation for orientations
    } smooth_trajectory;

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
    explicit Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    // Operator overloads
    Point3D operator+(const Point3D &other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D operator-(const Point3D &other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D operator*(float scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    Point3D operator/(float scalar) const {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    Point3D &operator+=(const Point3D &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Point3D &operator-=(const Point3D &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }
};

/**
 * @brief Joint angles for a single leg in degrees.
 */
struct JointAngles {
    float coxa, femur, tibia;
    explicit JointAngles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

/**
 * @brief IMU operation modes for different sensor capabilities
 */
enum IMUMode {
    IMU_MODE_RAW_DATA,    ///< Use raw sensor data with library algorithms
    IMU_MODE_FUSION,      ///< Use sensor's built-in sensor fusion
    IMU_MODE_ABSOLUTE_POS ///< Use sensor's absolute position calculations (e.g., BNO055)
};

/**
 * @brief Absolute position data from advanced IMUs
 */
struct IMUAbsoluteData {
    float absolute_roll, absolute_pitch, absolute_yaw;            ///< Absolute orientation in degrees
    float linear_accel_x, linear_accel_y, linear_accel_z;         ///< Linear acceleration (gravity removed)
    float quaternion_w, quaternion_x, quaternion_y, quaternion_z; ///< Orientation quaternion
    bool absolute_orientation_valid;                              ///< Whether absolute orientation is valid
    bool linear_acceleration_valid;                               ///< Whether linear acceleration is valid
    bool quaternion_valid;                                        ///< Whether quaternion data is valid
    uint8_t calibration_status;                                   ///< Overall calibration status (0-3, 3=fully calibrated)
    uint8_t system_status;                                        ///< System status from sensor
    uint8_t self_test_result;                                     ///< Self test result
};

/**
 * @brief Inertial measurement unit readings.
 */
struct IMUData {
    // Basic IMU data (always available)
    float roll, pitch, yaw;          ///< Euler angles in degrees
    float accel_x, accel_y, accel_z; ///< Raw acceleration in m/s²
    float gyro_x, gyro_y, gyro_z;    ///< Angular velocity in rad/s
    bool is_valid;                   ///< Basic data validity

    // Extended data for advanced IMUs
    IMUAbsoluteData absolute_data; ///< Absolute position data (when available)
    IMUMode mode;                  ///< Current operation mode
    bool has_absolute_capability;  ///< Whether IMU supports absolute positioning
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

    /** Set IMU operation mode (raw data, fusion, or absolute positioning). */
    virtual bool setIMUMode(IMUMode mode) = 0;

    /** Get current IMU operation mode. */
    virtual IMUMode getIMUMode() const = 0;

    /** Check if IMU supports absolute positioning (like BNO055). */
    virtual bool hasAbsolutePositioning() const = 0;

    /** Get calibration status for different sensor components (0-3, 3=fully calibrated). */
    virtual bool getCalibrationStatus(uint8_t *system, uint8_t *gyro, uint8_t *accel, uint8_t *mag) = 0;

    /** Trigger sensor self-test (if supported). */
    virtual bool runSelfTest() = 0;

    /** Reset sensor orientation (if supported). */
    virtual bool resetOrientation() = 0;

    /**
     * Update internal registers with sensor readings for parallel operation.
     * This method should trigger non-blocking reading of all IMU channels
     * and update internal data registers. Called by the locomotion
     * system during each update cycle for optimal sensor synchronization.
     * @return true if update was successful, false on error
     */
    virtual bool update() = 0;
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
    /**
     * Update internal registers with ADC readings using AdvancedAnalog DMA.
     * This method should trigger simultaneous reading of all FSR channels
     * and update the internal ADC value registers. Called by the locomotion
     * system during each update cycle for optimal sensor data synchronization.
     */
    virtual bool update() = 0;
};

class IServoInterface {
  public:
    virtual ~IServoInterface() = default;
    /** Initialize servo communication. */
    virtual bool initialize() = 0;

    /**
     * Check if any status flags are active that would prevent servo movement.
     * @param leg_index Index of the leg (0-5)
     * @param joint_index Index of the joint within leg (0-2: coxa, femur, tibia)
     * @return true if any blocking flags are active, false if servo is ready for movement
     */
    virtual bool hasBlockingStatusFlags(int leg_index, int joint_index) = 0;

    /**
     * Set a joint's angular position and velocity simultaneously.
     * This is the primary method for servo control - both parameters must be set together.
     * @param leg_index Index of the leg (0-5)
     * @param joint_index Index of the joint within leg (0-2: coxa, femur, tibia)
     * @param angle Target angular position in degrees
     * @param speed Movement velocity/speed for reaching the target position
     * @return true if command was successfully sent to servo
     */
    virtual bool setJointAngleAndSpeed(int leg_index, int joint_index, float angle, float speed) = 0;

    /** Retrieve the current joint angle. */
    virtual float getJointAngle(int leg_index, int joint_index) = 0;

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

    /** Get leg origin position in robot frame. */
    Point3D getLegOrigin(int leg) const;

  private:
    const Parameters &params;
    // DH parameter table: [leg][joint][param] where param = [a, alpha, d, theta_offset]
    float dh_transforms[NUM_LEGS][DOF_PER_LEG][4];
};

#endif // MODEL_H
