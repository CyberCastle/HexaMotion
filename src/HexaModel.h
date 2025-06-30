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

// Numerical differentiation step for Jacobians
#define JACOBIAN_DELTA 0.001f

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

    // Joint angle sign multipliers for hardware adaptation
    float angle_sign_coxa = 1.0f;  ///< Sign multiplier for coxa joint output (+1.0 or -1.0 to match servo direction)
    float angle_sign_femur = 1.0f; ///< Sign multiplier for femur joint output (+1.0 or -1.0 to match servo direction)
    float angle_sign_tibia = 1.0f; ///< Sign multiplier for tibia joint output (+1.0 or -1.0 to match servo direction)

    bool use_custom_dh_parameters = false; ///< Use custom Denavit-Hartenberg parameters
    /**
     * @brief DH parameter table for each leg.
     * The first entry represents the fixed base transform.
     */
    float dh_parameters[NUM_LEGS][DOF_PER_LEG + 1][4];

    Eigen::Vector3f imu_calibration_offset;
    float fsr_touchdown_threshold;
    float fsr_liftoff_threshold;
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
        uint8_t max_iterations = IK_DEFAULT_MAX_ITERATIONS; ///< Maximum iterations for RobotModel::inverseKinematics
        float pos_threshold_mm = 0.5f;
        bool use_damping = true;
        float damping_lambda = 30.0f;
        bool clamp_joints = true;
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

    bool operator==(const Point3D &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }

    bool operator!=(const Point3D &other) const {
        return !(*this == other);
    }

    float norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    Point3D normalized() const {
        float n = norm();
        if (n > 0) {
            return Point3D(x / n, y / n, z / n);
        }
        return Point3D(0, 0, 0);
    }
};

/**
 * @brief Pose structure with position and orientation (equivalent to OpenSHC's Pose)
 */
struct Pose {
    Point3D position;
    Eigen::Quaternionf rotation;

    explicit Pose(const Point3D &pos = Point3D(), const Eigen::Quaternionf &rot = Eigen::Quaternionf::Identity())
        : position(pos), rotation(rot) {}

    explicit Pose(const Point3D &pos, const Eigen::Vector3f &euler_angles_deg)
        : position(pos) {
        Eigen::Vector3f euler_rad = euler_angles_deg * M_PI / 180.0f;
        rotation = Eigen::AngleAxisf(euler_rad.z(), Eigen::Vector3f::UnitZ()) *
                   Eigen::AngleAxisf(euler_rad.y(), Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(euler_rad.x(), Eigen::Vector3f::UnitX());
    }

    static Pose Identity() {
        return Pose(Point3D(), Eigen::Quaternionf::Identity());
    }

    bool operator==(const Pose &other) const {
        return (position == other.position && rotation.isApprox(other.rotation));
    }

    bool operator!=(const Pose &other) const {
        return !(*this == other);
    }

    /**
     * Transform a pose by this pose (equivalent to OpenSHC's transform method)
     */
    Pose transform(const Eigen::Matrix4f &transform_matrix) const {
        Eigen::Vector4f pos_homogeneous(position.x, position.y, position.z, 1.0f);
        Eigen::Vector4f transformed_pos = transform_matrix * pos_homogeneous;

        Eigen::Matrix3f rot_matrix = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaternionf transformed_rot = Eigen::Quaternionf(rot_matrix) * rotation;

        return Pose(Point3D(transformed_pos.x(), transformed_pos.y(), transformed_pos.z()), transformed_rot);
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

    /** Compute inverse kinematics for a leg using a heuristic start guess. */
    JointAngles inverseKinematics(int leg, const Point3D &p) const;
    /**
     * @brief Compute inverse kinematics starting from current joint angles.
     * @param leg Leg index.
     * @param current_angles Current joint angles used as initial guess.
     * @param target Desired tip position in robot frame.
     */
    JointAngles inverseKinematicsCurrent(int leg, const JointAngles &current_angles,
                                         const Point3D &target) const;
    /** Compute forward kinematics for a leg. */
    Point3D forwardKinematics(int leg, const JointAngles &q) const;
    /** Analytic Jacobian for a leg. */
    Eigen::Matrix3f analyticJacobian(int leg, const JointAngles &q) const;
    /** Numerical Jacobian calculation. */
    Eigen::Matrix3f calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const;
    /** Homogeneous transform for a full leg chain. */
    Eigen::Matrix4d legTransform(int leg, const JointAngles &q) const;
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

    /**
     * @brief Get pose in robot frame (equivalent to OpenSHC's getPoseRobotFrame)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @param leg_frame_pose Pose relative to leg frame (default: identity)
     * @return Pose transformed to robot frame
     */
    Pose getPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &leg_frame_pose = Pose::Identity()) const;

    /**
     * @brief Get pose in leg frame (equivalent to OpenSHC's getPoseJointFrame)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @param robot_frame_pose Pose relative to robot frame (default: identity)
     * @return Pose transformed to leg frame
     */
    Pose getPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose = Pose::Identity()) const;

    /**
     * @brief Get tip pose in robot frame (equivalent to OpenSHC's tip getPoseRobotFrame)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @param tip_frame_pose Pose relative to tip frame (default: identity)
     * @return Tip pose transformed to robot frame
     */
    Pose getTipPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &tip_frame_pose = Pose::Identity()) const;

    /**
     * @brief Get tip pose in leg frame (equivalent to OpenSHC's tip getPoseTipFrame)
     * @param leg_index Index of the leg
     * @param joint_angles Current joint angles
     * @param robot_frame_pose Pose relative to robot frame (default: identity)
     * @return Tip pose transformed to leg frame
     */
    Pose getTipPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose = Pose::Identity()) const;

    /** Get the position of the leg base (without joint transformations) */
    Point3D getLegBasePosition(int leg_index) const;

  private:
    const Parameters &params;
    // DH parameter table: [leg][joint][param] where param = [a, alpha, d, theta_offset]
    // The first entry stores the fixed base transform for the leg
    float dh_transforms[NUM_LEGS][DOF_PER_LEG + 1][4];

    JointAngles solveIK(int leg, const Point3D &local_target, JointAngles current) const;
};

#endif // MODEL_H
