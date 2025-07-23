#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

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
    double hexagon_radius;
    double coxa_length;
    double femur_length;
    double tibia_length;

    double robot_height;
    double standing_height = 150; //< Default standing height in mm
    double height_offset = 0.0f;  //< structural body height offset
    double robot_weight;
    Eigen::Vector3d center_of_mass;

    double coxa_angle_limits[2];
    double femur_angle_limits[2];
    double tibia_angle_limits[2];

    // Joint angle sign multipliers for hardware adaptation
    double angle_sign_coxa = 1.0f;  //< Sign multiplier for coxa joint output (+1.0 or -1.0 to match servo direction)
    double angle_sign_femur = 1.0f; //< Sign multiplier for femur joint output (+1.0 or -1.0 to match servo direction)
    double angle_sign_tibia = 1.0f; //< Sign multiplier for tibia joint output (+1.0 or -1.0 to match servo direction)

    // Enable FSR contact detection
    bool use_fsr_contact = false;

    bool use_custom_dh_parameters = false; //< Use custom Denavit-Hartenberg parameters
    /**
     * @brief DH parameter table for each leg.
     * The first entry represents the fixed base transform.
     */
    double dh_parameters[NUM_LEGS][DOF_PER_LEG + 1][4];

    Eigen::Vector3d imu_calibration_offset;
    double fsr_touchdown_threshold;
    double fsr_liftoff_threshold;
    double fsr_max_pressure;

    double max_velocity;
    double max_angular_velocity;
    double stability_margin;
    double control_frequency;
    double default_servo_speed = SERVO_SPEED_DEFAULT; //< Default servo movement speed (0.1-3.0, where 1.0 is normal speed)

    /**
     * @brief Smooth trajectory configuration for pose updates.
     * Equivalent to OpenSHC's trajectory interpolation system.
     */
    struct SmoothTrajectoryConfig {
        bool use_current_servo_positions = false;          //< Use current servo positions as starting point for trajectories (OpenSHC-style)
        bool enable_pose_interpolation = false;            //< Enable smooth pose interpolation between positions
        double interpolation_speed = MIN_SERVO_VELOCITY;   //< Interpolation speed factor (0.01-1.0, where 0.1 is smooth)
        double position_tolerance_mm = POSITION_TOLERANCE; //< Position tolerance for determining if servo has reached target
        uint8_t max_interpolation_steps = 20;              //< Maximum steps for pose interpolation
        bool use_quaternion_slerp = true;                  //< Use spherical interpolation for orientations
    } smooth_trajectory;

    /**
     * @brief Inverse kinematics solver settings.
     */
    struct IKConfig {
        uint8_t max_iterations = IK_DEFAULT_MAX_ITERATIONS; //< Maximum iterations for RobotModel::inverseKinematics
        double pos_threshold_mm = 0.5f;
        bool use_damping = true;
        double damping_lambda = 30.0f;
        bool clamp_joints = true;
    } ik;

    /**
     * @brief Body compensation filter parameters.
     */
    struct BodyCompConfig {
        bool enable = true;
        double kp = 0.6f;
        double lp_alpha = 0.10f;
        double max_tilt_deg = 12.0f;
    } body_comp;

    // Gait step calculation factors
    /**
     * @brief Factors for calculating gait step length and height.
     */
    struct GaitFactors {
        // Step length as percentage of leg reach
        double tripod_length_factor = 0.35f;      // 35% for speed
        double wave_length_factor = 0.25f;        // 25% for stability
        double ripple_length_factor = 0.30f;      // 30% for balance
        double metachronal_length_factor = 0.28f; // 28% for smooth motion
        double adaptive_length_factor = 0.30f;    // 30% base for adaptation
        double amble_length_factor = 0.40f;       // 40% for maximum speed

        // Step height as percentage of robot height
        double tripod_height_factor = 0.30f;      // 30% for obstacle clearance
        double wave_height_factor = 0.20f;        // 20% for stability
        double ripple_height_factor = 0.25f;      // 25% for balance
        double metachronal_height_factor = 0.25f; // 25% for smooth motion
        double adaptive_height_factor = 0.25f;    // 25% base for adaptation
        double amble_height_factor = 0.35f;       // 35% for maximum clearance

        // Safety limits as percentage of leg reach/height
        double min_length_factor = 0.12f; // 12% minimum step length
        double max_length_factor = 0.50f; // 50% maximum step length
        double min_height_factor = 0.12f; // 12% minimum step height
        double max_height_factor = 0.40f; // 40% maximum step height
    } gait_factors;

    /**
     * @brief Dynamic gait timing configuration (OpenSHC equivalent)
     * Controls the calculation of swing and stance iterations based on frequency and time_delta
     */
    struct DynamicGaitConfig {
        // Phase timing parameters (equivalent to OpenSHC's gait.yaml)
        double stance_phase = 2.0; // Stance phase duration (OpenSHC default: 2.0)
        double swing_phase = 2.0;  // Swing phase duration (OpenSHC default: 2.0)
        double phase_offset = 2.0; // Phase offset between legs (OpenSHC default: 2.0)

        // Frequency and timing parameters
        double frequency = 1.0;   // Control frequency (Hz, OpenSHC default: 1.0)
        double time_delta = 0.01; // Time step (seconds, OpenSHC default: 0.01)

        // Dynamic iteration calculation parameters
        bool enable_dynamic_iterations = true; // Enable OpenSHC-style dynamic iteration calculation
        double swing_period_factor = 1.0;      // Swing period scaling factor
        double stance_period_factor = 1.0;     // Stance period scaling factor

        // Iteration limits for safety
        uint16_t min_swing_iterations = 5;    // Minimum swing iterations
        uint16_t max_swing_iterations = 100;  // Maximum swing iterations
        uint16_t min_stance_iterations = 5;   // Minimum stance iterations
        uint16_t max_stance_iterations = 100; // Maximum stance iterations

        // Duty factor calculation (OpenSHC equivalent)
        double duty_factor = 0.5; // Duty factor (0.5 = 50% stance, 50% swing)

        // Step period calculation
        double step_period = 4.0; // Total step period (stance + swing)
    } dynamic_gait;

    // Tipo de gait seleccionado (OpenSHC compatible)
    std::string gait_type;

    // Gait phase offset multiplier for tripod gait synchronization
    double offset_multiplier = 0.5; // Default offset multiplier for tripod gait phase synchronization
};

enum GaitType {
    NO_GAIT,
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

enum StepPhase {
    STANCE_PHASE,
    SWING_PHASE,
    LIFT_PHASE,
    TOUCHDOWN_PHASE
};

/**
 * @brief 3D coordinate in millimeters.
 */
struct Point3D {
    double x, y, z;
    explicit Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    // Operator overloads
    Point3D operator+(const Point3D &other) const {
        return Point3D(x + other.x, y + other.y, z + other.z);
    }

    Point3D &operator+=(const Point3D &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Point3D operator-(const Point3D &other) const {
        return Point3D(x - other.x, y - other.y, z - other.z);
    }

    Point3D operator*(double scalar) const {
        return Point3D(x * scalar, y * scalar, z * scalar);
    }

    Point3D operator/(double scalar) const {
        return Point3D(x / scalar, y / scalar, z / scalar);
    }

    bool operator==(const Point3D &other) const {
        return (x == other.x && y == other.y && z == other.z);
    }

    bool operator!=(const Point3D &other) const {
        return !(*this == other);
    }

    double norm() const {
        return sqrt(x * x + y * y + z * z);
    }

    Point3D normalized() const {
        double n = norm();
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
    Eigen::Quaterniond rotation;

    explicit Pose(const Point3D &pos = Point3D(), const Eigen::Quaterniond &rot = Eigen::Quaterniond::Identity())
        : position(pos), rotation(rot) {}

    explicit Pose(const Point3D &pos, const Eigen::Vector3d &euler_angles_deg)
        : position(pos) {
        Eigen::Vector3d euler_rad = euler_angles_deg * M_PI / 180.0f;
        rotation = Eigen::AngleAxisd(euler_rad.z(), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(euler_rad.y(), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(euler_rad.x(), Eigen::Vector3d::UnitX());
    }

    static Pose Identity() {
        return Pose(Point3D(), Eigen::Quaterniond::Identity());
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
    Pose transform(const Eigen::Matrix4d &transform_matrix) const {
        Eigen::Vector4d pos_homogeneous(position.x, position.y, position.z, 1.0);
        Eigen::Vector4d transformed_pos = transform_matrix * pos_homogeneous;

        Eigen::Matrix3d rot_matrix = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaterniond transformed_rot(rot_matrix);
        Eigen::Quaterniond result_rot = transformed_rot * rotation.cast<double>();

        return Pose(Point3D(transformed_pos.x(),
                            transformed_pos.y(),
                            transformed_pos.z()),
                    result_rot.cast<double>());
    }

    /**
     * Transform a vector into this pose's reference frame (equivalent to OpenSHC's transformVector)
     */
    Point3D transformVector(const Point3D &vec) const {
        Eigen::Vector3d eigen_vec(vec.x, vec.y, vec.z);
        Eigen::Vector3d eigen_pos(position.x, position.y, position.z);
        Eigen::Vector3d transformed = eigen_pos + rotation.cast<double>()._transformVector(eigen_vec);
        return Point3D(transformed.x(), transformed.y(), transformed.z());
    }

    /**
     * Transform a vector from this pose's reference frame (equivalent to OpenSHC's inverseTransformVector)
     */
    Point3D inverseTransformVector(const Point3D &vec) const {
        return inverse().transformVector(vec);
    }

    /**
     * Get the inverse of this pose (equivalent to OpenSHC's ~ operator)
     */
    Pose inverse() const {
        Eigen::Quaterniond inv_rotation = rotation.cast<double>().conjugate();
        Eigen::Vector3d eigen_pos(position.x, position.y, position.z);
        Eigen::Vector3d inv_position = inv_rotation._transformVector(-eigen_pos);
        return Pose(Point3D(inv_position.x(), inv_position.y(), inv_position.z()), inv_rotation.cast<double>());
    }

    /**
     * Add another pose to this pose (equivalent to OpenSHC's addPose)
     */
    Pose addPose(const Pose &other) const {
        Point3D new_position = transformVector(other.position);
        Eigen::Quaterniond new_rotation = rotation.cast<double>() * other.rotation.cast<double>();
        return Pose(new_position, new_rotation.cast<double>());
    }

    /**
     * Remove another pose from this pose (equivalent to OpenSHC's removePose)
     */
    Pose removePose(const Pose &other) const {
        Point3D new_position = transformVector(Point3D(-other.position.x, -other.position.y, -other.position.z));
        Eigen::Quaterniond new_rotation = rotation.cast<double>() * other.rotation.cast<double>().inverse();
        return Pose(new_position, new_rotation.cast<double>());
    }

    /**
     * Interpolate between this pose and target pose (equivalent to OpenSHC's interpolate)
     */
    Pose interpolate(double control_input, const Pose &target_pose) const {
        Point3D new_position = position * (1.0 - control_input) + target_pose.position * control_input;
        Eigen::Quaterniond new_rotation = rotation.cast<double>().slerp(control_input, target_pose.rotation.cast<double>());
        return Pose(new_position, new_rotation.cast<double>());
    }
};

/**
 * @brief Joint angles for a single leg in radians.
 */
struct JointAngles {
    double coxa, femur, tibia;
    explicit JointAngles(double c = 0, double f = 0, double t = 0) : coxa(c), femur(f), tibia(t) {}
};

/**
 * @brief IMU operation modes for different sensor capabilities
 */
enum IMUMode {
    IMU_MODE_RAW_DATA,    //< Use raw sensor data with library algorithms
    IMU_MODE_FUSION,      //< Use sensor's built-in sensor fusion
    IMU_MODE_ABSOLUTE_POS //< Use sensor's absolute position calculations (e.g., BNO055)
};

/**
 * @brief Absolute position data from advanced IMUs
 */
struct IMUAbsoluteData {
    double absolute_roll, absolute_pitch, absolute_yaw;            //< Absolute orientation in degrees
    double linear_accel_x, linear_accel_y, linear_accel_z;         //< Linear acceleration (gravity removed)
    double quaternion_w, quaternion_x, quaternion_y, quaternion_z; //< Orientation quaternion
    bool absolute_orientation_valid;                               //< Whether absolute orientation is valid
    bool linear_acceleration_valid;                                //< Whether linear acceleration is valid
    bool quaternion_valid;                                         //< Whether quaternion data is valid
    uint8_t calibration_status;                                    //< Overall calibration status (0-3, 3=fully calibrated)
    uint8_t system_status;                                         //< System status from sensor
    uint8_t self_test_result;                                      //< Self test result
};

/**
 * @brief Inertial measurement unit readings.
 */
struct IMUData {
    // Basic IMU data (always available)
    double roll, pitch, yaw;          //< Euler angles in degrees
    double accel_x, accel_y, accel_z; //< Raw acceleration in m/s²
    double gyro_x, gyro_y, gyro_z;    //< Angular velocity in rad/s
    bool is_valid;                    //< Basic data validity

    // Extended data for advanced IMUs
    IMUAbsoluteData absolute_data; //< Absolute position data (when available)
    IMUMode mode;                  //< Current operation mode
    bool has_absolute_capability;  //< Whether IMU supports absolute positioning
};

/**
 * @brief Force sensing resistor data for a foot.
 */
struct FSRData {
    double pressure;
    bool in_contact;
    double contact_time;
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
    virtual double getRawReading(int leg_index) = 0;
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
    virtual bool setJointAngleAndSpeed(int leg_index, int joint_index, double angle, double speed) = 0;

    /** Retrieve the current joint angle. */
    virtual double getJointAngle(int leg_index, int joint_index) = 0;

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

    /** Compute inverse kinematics for a leg using a heuristic start guess (Global coordinates). */
    JointAngles inverseKinematicsGlobalCoordinates(int leg, const Point3D &p) const;

    /**
     * @brief Estimate initial joint angles for inverse kinematics based on target position
     * This method provides a more intelligent initial guess than the basic atan2 approach,
     * taking into account the target position and leg geometry to estimate reasonable
     * starting angles for the IK solver.
     *
     * @param leg Leg index (0-5)
     * @param target_position Target position in global robot coordinates
     * @return Estimated initial joint angles for IK solver
     */
    JointAngles estimateInitialAngles(int leg, const Point3D &target_position) const;
    /**
     * @brief Compute inverse kinematics starting from current joint angles (Global coordinates).
     * @param leg Leg index.
     * @param current_angles Current joint angles used as initial guess.
     * @param target Desired tip position in robot frame.
     */
    JointAngles inverseKinematicsCurrentGlobalCoordinates(int leg, const JointAngles &current_angles,
                                                          const Point3D &target) const;
    /** Compute forward kinematics for a leg (Global coordinates). */
    Point3D forwardKinematicsGlobalCoordinates(int leg, const JointAngles &q) const;

    /** Numerical Jacobian calculation. */
    Eigen::Matrix3d calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const;
    /** Homogeneous transform for a full leg chain. */
    Eigen::Matrix4d legTransform(int leg, const JointAngles &q) const;
    /** Verify if joint angles are within defined limits. */
    bool checkJointLimits(int leg, const JointAngles &q) const;
    /** Clamp angle within limits. */
    double constrainAngle(double angle, double min_angle, double max_angle) const;
    /** Normalize angle to [-pi, pi] radians. */
    double normalizeAngle(double angle_rad) const;
    /** Validate parameter consistency. */
    bool validate() const;
    /**
     * \brief Calculate minimal and maximal body height based on joint limits.
     * \return Pair {min_height, max_height} in millimeters.
     */
    std::pair<double, double> calculateHeightRange() const;
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
     * @brief Get leg reach distance.
     * @return Maximum reach distance
     */
    double getLegReach() const;

    /** Get the DH position of the leg base (without joint transformations) */
    Point3D getLegBasePosition(int leg_index) const;

    /** Get the base angle offset for a specific leg in radians */
    double getLegBaseAngleOffset(int leg_index) const;

    /**
     * Calculate target position based on current position (OpenSHC-style)
     * This method transforms a target position from the current pose's reference frame
     * and uses inverseKinematicsCurrent to calculate the required joint angles.
     *
     * @param leg The leg index (0-5)
     * @param current_angles Current joint angles for the leg
     * @param current_pose Current robot pose
     * @param target_in_current_frame Target position in current pose's reference frame
     * @return Joint angles to reach the target position
     */
    JointAngles calculateTargetFromCurrentPosition(int leg, const JointAngles &current_angles,
                                                   const Pose &current_pose, const Point3D &target_in_current_frame) const;

    /**
     * Calculate target position based on current position with default stance pose (OpenSHC-style)
     * This method calculates the target position by transforming the default stance pose
     * from the current pose's reference frame, similar to OpenSHC's pose controller.
     *
     * @param leg The leg index (0-5)
     * @param current_angles Current joint angles for the leg
     * @param current_pose Current robot pose
     * @param default_stance_pose Default stance pose in world frame
     * @return Joint angles to reach the transformed default stance position
     */
    JointAngles calculateTargetFromDefaultStance(int leg, const JointAngles &current_angles,
                                                 const Pose &current_pose, const Pose &default_stance_pose) const;

    /**
     * @brief Apply inverse kinematics using local leg coordinates (OpenSHC-style)
     * This method follows OpenSHC's approach by transforming global coordinates to local leg frame
     * before solving IK, ensuring symmetry is maintained.
     *
     * @param leg Leg index (0-5)
     * @param global_target Target position in global robot coordinates
     * @param current_angles Current joint angles for initial guess
     * @return Joint angles to reach the target position
     */
    JointAngles solveIKLocalCoordinates(int leg, const Point3D &global_target,
                                        const JointAngles &current_angles) const;

    /**
     * @brief Transform global position to local leg coordinates (OpenSHC-style)
     * This method transforms a position from global robot coordinates to local leg coordinates,
     * following OpenSHC's getPoseJointFrame approach.
     *
     * @param leg Leg index (0-5)
     * @param global_position Position in global robot coordinates
     * @param current_angles Current joint angles for the transformation
     * @return Position in local leg coordinates
     */
    Point3D transformGlobalToLocalCoordinates(int leg, const Point3D &global_position,
                                              const JointAngles &current_angles) const;

    /**
     * @brief Transform local position to global robot coordinates (OpenSHC-style)
     * This method transforms a position from local leg coordinates to global robot coordinates,
     * following OpenSHC's getPoseRobotFrame approach.
     *
     * @param leg Leg index (0-5)
     * @param local_position Position in local leg coordinates
     * @param current_angles Current joint angles for the transformation
     * @return Position in global robot coordinates
     */
    Point3D transformLocalToGlobalCoordinates(int leg, const Point3D &local_position,
                                              const JointAngles &current_angles) const;

    /**
     * @brief Make a position reachable by constraining it to leg workspace (OpenSHC-style)
     * This function follows OpenSHC's approach to automatically adjust positions that are
     * outside the leg's workspace to be within reachable bounds.
     *
     * @param leg_index Index of the leg (0-5)
     * @param reference_tip_position Target position that may be outside workspace
     * @return Adjusted position that is guaranteed to be within leg workspace
     */
    Point3D makeReachable(int leg_index, const Point3D &reference_tip_position) const;

    /**
     * @brief Complete IK solver with position delta calculation and joint limit optimization
     * This function implements a robust IK method that:
     * 1. Calculate position delta in leg frame (desired - current)
     * 2. Apply DLS-based IK with joint limit cost function
     * 3. Update joint positions with velocity clamping
     *
     * @param leg Leg index
     * @param current_tip_pose Current tip position in global coordinates
     * @param desired_tip_pose Desired tip position in global coordinates
     * @param current_angles Current joint angles
     * @param time_delta Time delta for velocity calculation (typically control frequency)
     * @return Updated joint angles after advanced IK
     */
    JointAngles applyAdvancedIK(int leg, const Point3D &current_tip_pose, const Point3D &desired_tip_pose,
                                const JointAngles &current_angles, double time_delta = 0.02) const;

    /**
     * @brief Joint limit cost function and gradient calculation
     * Implements cost function optimization for joint position and velocity limits
     *
     * @param current_angles Current joint angles
     * @param joint_velocities Joint velocities
     * @param leg Leg index
     * @return Combined cost gradient vector
     */
    Eigen::Vector3d calculateJointLimitCostGradient(const JointAngles &current_angles,
                                                    const Eigen::Vector3d &joint_velocities, int leg) const;

    /**
     * @brief Core IK solver method with delta vector input (internal)
     * Implements DLS-based IK for position delta calculation
     *
     * @param leg Leg index
     * @param delta 6D delta vector (position + rotation, though we use only position)
     * @param current_angles Current joint angles
     * @return Joint position delta
     */
    Eigen::Vector3d solveDeltaIK(int leg, const Eigen::MatrixXd &delta, const JointAngles &current_angles) const;
    std::vector<Eigen::Matrix4d> buildDHTransforms(int leg, const JointAngles &q) const;

  private:
    Parameters params;
    // DH parameter table: [leg][joint][param] where param = [a, alpha, d, theta_offset]
    // The first entry stores the fixed base transform for the leg
    double dh_transforms[NUM_LEGS][DOF_PER_LEG + 1][4];

    // Internal joint limits in radians (converted from degrees in constructor)
    double coxa_angle_limits_rad[2];
    double femur_angle_limits_rad[2];
    double tibia_angle_limits_rad[2];
    double max_angular_velocity_rad;
    double body_comp_max_tilt_rad;

    JointAngles solveIK(int leg, const Point3D &local_target, JointAngles current) const;

    // Helper methods to reduce code duplication
    Point3D transformGlobalToLocalLegCoordinates(int leg, const Point3D &global_target) const;
    void clampJointAngles(JointAngles &angles) const;
};

#endif // ROBOT_MODEL_H
