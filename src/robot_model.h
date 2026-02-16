#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "gait_types.h" // Shared gait type enumeration
#include "hexamotion_constants.h"
#include "math_utils.h"
#include "precision_config.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <cmath>
#include <limits>
#include <memory>
#include <utility>

// Forward declaration para evitar dependencias circulares
class WorkspaceAnalyzer;
struct ValidationConfig;

/**
 * @brief Joint pose angles for one leg in radians.
 */
struct JointPoseAngles {
    double coxa = 0.0;
    double femur = 0.0;
    double tibia = 0.0;
};

/**
 * @brief Robot configuration parameters.
 */
struct Parameters {
    Parameters()
        : hexagon_radius(0), coxa_length(0), femur_length(0), tibia_length(0),
          robot_height(0),
          coxa_angle_limits{0, 0}, femur_angle_limits{0, 0}, tibia_angle_limits{0, 0},
          dh_parameters{},
          max_velocity(0), max_angular_velocity(0), stability_margin(0) {
        for (int i = 0; i < NUM_LEGS; ++i) {
            packed_pose_joints[i] = {-1.571, 1.900, 1.200};
            unpacked_pose_joints[i] = {0.000, 0.785, -1.138};
        }
    }

    double hexagon_radius;
    double coxa_length;
    double femur_length;
    double tibia_length;

    double robot_height;
    double standing_height = 150;        //< Default standing height in mm
    double height_offset = 0.0f;         //< structural body height offset
    double default_height_offset = 0.0f; //< Default height offset when all joint angles are 0°

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

    // Configured packed/unpacked joint poses (OpenSHC YAML parity moved to Parameters).
    // All values are in radians in the robot model frame.
    bool use_configured_packed_unpacked_poses = true; //< Enable explicit packed/unpacked joint pose targets
    JointPoseAngles packed_pose_joints[NUM_LEGS];     //< Packed target pose per leg
    JointPoseAngles unpacked_pose_joints[NUM_LEGS];   //< Unpacked (ready) target pose per leg

    /**
     * @brief DH parameter table for each leg.
     * The first entry represents the fixed base transform.
     */
    double dh_parameters[NUM_LEGS][DOF_PER_LEG + 1][4];

    // FSR contact filtering thresholds (used in LocomotionSystem::updateLegStates)
    // fsr_touchdown_threshold: minimum historical rolling average to consider contact (hysteresis enter)
    // fsr_liftoff_threshold: maximum historical rolling average to consider release (hysteresis exit)
    // fsr_min_pressure: minimum normalized/raw pressure/average to validate physical contact and reject false positives
    double fsr_touchdown_threshold = 0.7; //< Average contact value (0-1) to switch to STANCE
    double fsr_liftoff_threshold = 0.3;   //< Average contact value (0-1) to switch to SWING
    double fsr_min_pressure = 0.05;       //< Minimum normalized value (0-1) to trust reported contact (legacy raw=10 maps ≈0.05)
    double fsr_max_pressure = 0.9;        //< Maximum expected normalized pressure (0-1). 1.0 = saturated/full contact (used for clamping/validation)

    double max_velocity;
    double max_angular_velocity;
    double overshoot_stride_fraction = DEFAULT_OVERSHOOT_STRIDE_FRACTION;   //< Max fraction of stride dedicated to overshoot damping
    double min_effective_stride_ratio = DEFAULT_MIN_EFFECTIVE_STRIDE_RATIO; //< Minimum stride fraction preserved after overshoot deduction
    double stability_margin;

    // Global gait tempo (Hz). Used by GaitConfiguration::generateStepCycle() to derive the
    // nominal (pre‑normalization) cycle duration: raw_iterations ≈ (1 / step_frequency) / time_delta.
    // The final StepCycle period is then normalized to an even multiple of (stance_phase + swing_phase)
    // to preserve exact whole‑number iteration counts per sub‑phase. Change this to speed up or slow
    // down all gaits uniformly. Runtime changes require regenerating gait configs (factory) before use.
    double step_frequency = 1.0; // Default OpenSHC tempo (1 Hz)

    // Unified control loop timestep (seconds). Single source of discrete integration resolution for:
    //  * Gait iteration counts (StepCycle normalization)
    //  * Velocity/acceleration limiting and stride integration
    //  * IK velocity-based adjustments and timing-dependent filters
    // Deterministic fixed step chosen for reproducibility.
    // Adjust only if the actual loop rate changes; otherwise leave at 0.02 (50 Hz).
    double time_delta = 0.02;                         // 50 Hz control loop
    double default_servo_speed = SERVO_SPEED_DEFAULT; //< Default servo movement speed (0.1-3.0, where 1.0 is normal speed)
    // Enable kinematic integration of body translation & yaw (simulation/testing)
    bool enable_body_translation = false; //< When true, LocomotionSystem::update() integrates body_position & yaw from commanded velocities

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
        // PID gains for IMU posing (OpenSHC rotation_pid_gains equivalent)
        double imu_pid_kp = 1.0; //< Proportional gain for rotation correction
        double imu_pid_ki = 0.0; //< Integral gain (absement) for rotation correction
        double imu_pid_kd = 0.0; //< Derivative gain (angular velocity) for rotation correction
    } body_comp;

    /**
     * @brief Manual leg manipulation parameters (OpenSHC equivalent).
     */
    struct ManualLegConfig {
        double max_translation_velocity = 200.0; //< Max tip translation speed in mm/s (OpenSHC: max_translation_velocity)
        double max_rotation_velocity = 1.0;      //< Max joint rotation speed in rad/s (OpenSHC: max_rotation_velocity)
        bool joint_control = true;               //< true = joint_control mode, false = tip_control mode (OpenSHC: leg_manipulation_mode)
    } manual_leg;

    // OpenSHC-equivalent body posing runtime flags and velocity caps.
    bool manual_posing = true;
    bool inclination_posing = false;
    bool imu_posing = false;
    bool auto_posing = false;
    bool gravity_aligned_tips = false;
    double max_translation_velocity = 50.0; //< mm/s
    double max_rotation_velocity = 0.2;     //< rad/s
    double pose_frequency = -1.0;           //< Hz, -1 sync to gait cycle
    int pose_phase_length = 4;
    double time_to_start = 6.0; //< directStartup duration in seconds

    // Tipo de gait seleccionado (OpenSHC compatible)
    std::string gait_type;

    // Walkspace overlap control (OpenSHC equivalent)
    bool overlapping_walkspaces = false; // Flag denoting if walkspaces are allowed to overlap (default: false, same as OpenSHC)

    // Gait continuity control: when true, preserve the swing end (touchdown) pose as stance origin instead of
    // resetting to the default tip pose. This yields smoother, continuous trajectories (OpenSHC-style continuity)
    // at the cost of potential long-term drift. When false, an anti-drift policy resets (or blends toward) the
    // calibrated default tip pose at the start of stance for deterministic repeatability.
    bool preserve_swing_end_pose = true; // true = continuity (default), false = anti-drift (uses hybrid reset logic below)

    // --- Drift management thresholds ---
    // Shared by both continuity and hybrid reset modes. In continuity (preserve_swing_end_pose=true) these values
    // gate the lateral residual cleanup executed at stance entry so that rectilinear commands remain tightly
    // synchronized while allowing rotation commands to keep their intentional offset. When continuity is disabled,
    // the same thresholds also decide whether to blend toward or snap back to the calibrated default tip pose.
    // Distances expressed in millimeters.
    double drift_soft_threshold_mm = 2.0; //< Within this distance: apply soft blend (or mild lateral cleanup)
    double drift_hard_threshold_mm = 8.0; //< Beyond this distance: force hard reset / correction snap
    double drift_soft_blend_alpha = 0.5;  //< Blend factor (0..1) for soft correction (0.5 = halfway to default)

#ifdef TESTING_ENABLED
    double drift_metrics_ema_alpha = 0.1;        //< Exponential moving average smoothing factor (0..1) for drift magnitude
    double drift_metrics_cap_mm = 500.0;         //< Cap for reported accumulated drift norm to avoid unbounded growth
    bool report_planar_vs_vertical_drift = true; //< When true, separate planar (XY) vs vertical (Z) drift in reports
    bool debug_fsr_transitions = false;          //< Log phase transitions driven by FSR contact
#endif

    // Workspace constraint toggle: when true (default) all target and intermediate tip poses are constrained
    // via WorkspaceAnalyzer to remain within geometric reach envelopes. When false, raw trajectories are used
    // (useful for debugging or external safety layers). Disabling can cause IK failures or unrealistic poses.
    bool enable_workspace_constrain = true;

    // === HexaMotion extension (NOT present in original OpenSHC) ===
    // Optional phase-end snap to reduce residual drift of foot trajectory endpoints when using
    // derivative (velocity) integration. Set enable_phase_end_snap=false to emulate OpenSHC exactly.
    bool enable_phase_end_snap = true;        //< Enable snapping foot to frozen target at phase end
    double phase_end_snap_tolerance_mm = 1.0; //< Distance tolerance (mm) for hard snap
    double phase_end_snap_alpha = 1.0;        //< Blend factor (1.0 hard snap, <1.0 partial correction)

    // --- Constraint tolerances ---
    // Tolerancia para preservar la altura exacta del plano de marcha al aplicar
    // el constriñimiento geométrico (evita deriva vertical artificial en touchdown)
    double walk_plane_z_tolerance_mm = WALK_PLANE_Z_TOLERANCE_MM;

    /**
     * @brief Global motion and workspace scaling factors used by higher-level controllers.
     *
     * These values previously lived as literal constants (e.g. 0.65, 0.9, 1.0) in controller logic. Exposing them here
     * allows runtime / configuration level tuning (same style as StartupNormalizationConfig) without touching core code.
     *
     * Usage notes:
     *  - collision_scale: when <= 0.0 the validation_config_.safety_margin_factor is used dynamically.
     *  - safety_margin: generic multiplier applied by controllers (e.g. servo speed clamping) for unified conservative tuning.
     *  - Keep values in a sane physical range (0.4 – 1.2) to avoid destabilizing stride / velocity estimations.
     */
    struct ScalingFactors {
        double linear_scale = 0.65;      //< Legacy linear scaling (replaces scattered WORKSPACE / WALKSPACE constants)
        double angular_scale = 1.0;      //< Angular scaling (kept at 1.0 unless deliberate reduction required)
        double workspace_scale = 0.65;   //< Conservative workspace envelope scaling
        double collision_scale = 0.0;    //< If <= 0 => derive from ValidationConfig::safety_margin_factor
        double velocity_scale = 0.9;     //< 10% safety margin for derived velocity limits
        double acceleration_scale = 1.0; //< Acceleration scaling (placeholder for future tuning)
        double safety_margin = 0.9;      //< Unified safety margin for servo speed / other conservative clamps
    } scaling;                           //< Instance accessible as params.scaling

    /**
     * @brief Workspace & morphology heuristic tuning factors.
     * All former hardcoded literals in WorkspaceAnalyzer moved here for external configurability.
     * Keep factors within physically meaningful ranges to avoid destabilizing gait generation.
     */
    struct WorkspaceTuning {
        // Stability & collision
        double stability_threshold_mm = 10.0; //< Min stability margin to be considered stable
        double min_leg_separation_mm = 50.0;  //< Minimum planar distance between adjacent leg tips

        // Morphology reach heuristics
        double morphology_cap_factor = 1.15;           //< Headroom over standing_horizontal_reach for max_reach cap
        double femur_up_range_factor = 0.85;           //< Upward (positive Z) reachable fraction of femur length
        double down_range_factor = 0.85;               //< Downward (negative Z) fraction of (femur+tibia)
        double leg_workspace_height_span_factor = 0.7; //< Percentage of total reach for +/- height span in cached workspace

        // Preferred reach buffer
        double preferred_min_reach_buffer_factor = 1.1; //< Multiplier over absolute min reach for preferred_min_reach

        // Collision avoidance iterative scaling (adjustForCollisionAvoidance)
        double collision_adjust_start_scale = 0.9; //< Initial radial scale attempt
        double collision_adjust_min_scale = 0.5;   //< Minimum radial scale attempt
        double collision_adjust_step = 0.1;        //< Decrement step per attempt
        double safe_scale_ratio = 0.7;             //< Fallback ratio of leg_reach when iterative scaling fails
    } workspace_tuning;                            //< params.workspace_tuning

    /**
     * @brief Admittance controller parameters (OpenSHC 1:1 equivalent).
     *
     * Replaces OpenSHC YAML: admittance_control, dynamic_stiffness, use_joint_effort,
     * integrator_step_time, virtual_mass, virtual_stiffness, virtual_damping_ratio,
     * force_gain, swing_stiffness_scaler, load_stiffness_scaler.
     */
    struct AdmittanceConfig {
        bool enable = false;                 //< Master toggle (OpenSHC: admittance_control)
        bool dynamic_stiffness = true;       //< Enable per-phase stiffness scaling (OpenSHC: dynamic_stiffness)
        bool use_joint_effort = false;       //< Use calculated (true) vs measured (false) tip force (OpenSHC: use_joint_effort)
        double integrator_step_time = 0.5;   //< Integration step time in seconds (OpenSHC default: 0.5)
        double virtual_mass = 10.0;          //< Virtual mass in kg (OpenSHC default: 10.0)
        double virtual_stiffness = 12.0;     //< Virtual spring stiffness (OpenSHC default: 12.0)
        double virtual_damping_ratio = 0.8;  //< Damping ratio ζ; actual damping = ζ·2·√(m·k) (OpenSHC default: 0.8)
        double force_gain = 0.1;             //< Force scaling gain (OpenSHC default: 0.1)
        double swing_stiffness_scaler = 0.1; //< Stiffness scaler for swing legs (OpenSHC default: 0.1)
        double load_stiffness_scaler = 5.0;  //< Stiffness scaler for loaded adjacent legs (OpenSHC default: 5.0)
    } admittance;                            //< params.admittance
};

// Centralized servo angle solution for standing height (previously in body_pose_config_factory)
struct CalculatedServoAngles {
    double coxa;  // Coxa servo angle (radians)
    double femur; // Femur servo angle (radians)
    double tibia; // Tibia servo angle (radians)
    bool valid;   // Solution validity flag
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
        Eigen::Vector3d euler_rad = euler_angles_deg * math_utils::degreesToRadians(1.0);
        rotation = Eigen::AngleAxisd(euler_rad.z(), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(euler_rad.y(), Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(euler_rad.x(), Eigen::Vector3d::UnitX());
    }

    static Pose Identity() {
        return Pose(Point3D(), Eigen::Quaterniond::Identity());
    }

    /**
     * @brief Create a sentinel undefined pose (NaN components).
     * @return Undefined pose instance.
     */
    static Pose Undefined() {
        double nan = std::numeric_limits<double>::quiet_NaN();
        Eigen::Quaterniond q(nan, nan, nan, nan);
        return Pose(Point3D(nan, nan, nan), q);
    }

    /**
     * @brief Check if pose components are finite and usable.
     * @return True if position and rotation are finite.
     */
    bool isValid() const {
        if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z)) {
            return false;
        }
        return std::isfinite(rotation.w()) && std::isfinite(rotation.x()) &&
               std::isfinite(rotation.y()) && std::isfinite(rotation.z());
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

    /**
     * Extended joint motion command including acceleration.
     * Implementations that do not natively support acceleration can ignore the parameter and
     * fallback to setJointAngleAndSpeed(). Default implementation delegates to that legacy method.
     * @param leg_index Index of the leg (0-5)
     * @param joint_index Joint index within leg (0-2)
     * @param angle Target angular position in degrees
     * @param speed Target (approximate) velocity or driver speed parameter
     * @param acceleration Optional acceleration limit (driver units or deg/s^2). May be ignored.
     * @return true if command accepted
     */
    virtual bool setJointAngleSpeedAccel(int leg_index, int joint_index,
                                         double angle, double speed, double acceleration) {
        (void)acceleration; // default: unused
        return setJointAngleAndSpeed(leg_index, joint_index, angle, speed);
    }

    /** Retrieve the current joint angle (radians). */
    virtual double getJointAngle(int leg_index, int joint_index) = 0;

    /** Retrieve the current joint velocity (rad/s). Default returns 0.0 if unsupported. */
    virtual double getJointVelocity(int leg_index, int joint_index) { return 0.0; }

    /** Retrieve the current joint effort/torque (driver units). Default returns 0.0 if unsupported. */
    virtual double getJointEffort(int leg_index, int joint_index) { return 0.0; }

    /** Check if a joint is currently moving. */
    virtual bool isJointMoving(int leg_index, int joint_index) = 0;
    /** Enable or disable torque on a joint. */
    virtual bool enableTorque(int leg_index, int joint_index, bool enable) = 0;

    /**
     * Batch command to set all joints' angles and speeds in one call.
     * Implementations may use a bus-level synchronous write to reduce latency and jitter.
     * Default returns false to indicate not supported.
     *
     * @param angles_deg Degrees, indexed [leg][joint] (coxa=0,femur=1,tibia=2)
     * @param speeds Speed multipliers or driver-native speed values [leg][joint]
     * @return true if batch command was sent, false to fallback to per-joint commands
     */
    virtual bool syncSetAllJointAnglesAndSpeeds(const double angles_deg[NUM_LEGS][DOF_PER_LEG],
                                                const double speeds[NUM_LEGS][DOF_PER_LEG]) {
        (void)angles_deg;
        (void)speeds;
        return false;
    }

    /**
     * Batch command to set all joints' angles, speeds and accelerations.
     * This extends syncSetAllJointAnglesAndSpeeds by adding an acceleration parameter.
     * Implementations that do not
     * natively support acceleration limits may ignore the parameter and delegate to
     * syncSetAllJointAnglesAndSpeeds(), mirroring the behaviour of
     * setJointAngleSpeedAccel which falls back to setJointAngleAndSpeed.
     *
     * Default implementation: delegates to syncSetAllJointAnglesAndSpeeds() and ignores
     * acceleration values, returning its result. Override for bus-level optimized
     * synchronous write including acceleration control.
     *
     * @param angles_deg Target joint angles in degrees [leg][joint].
     * @param speeds Target joint speeds or driver-native speed values [leg][joint].
     * @param accelerations Target joint accelerations (driver units or deg/s^2) [leg][joint].
     * @return true if batch command was sent, false otherwise (caller may fallback to per-joint calls).
     */
    virtual bool syncSetAllJointAnglesSpeedsAccels(const double angles_deg[NUM_LEGS][DOF_PER_LEG],
                                                   const double speeds[NUM_LEGS][DOF_PER_LEG],
                                                   const double accelerations[NUM_LEGS][DOF_PER_LEG]) {
        (void)accelerations; // default: unused (fallback)
        return syncSetAllJointAnglesAndSpeeds(angles_deg, speeds);
    }

    /**
     * @brief Refresh a small slice of servo health state without blocking the hot path.
     * @details Implementations should poll at most @p max_per_cycle servos per call and update
     *          an internal cache of fault/blocking flags. No heavy I/O should be performed
     *          elsewhere in the publish loop.
     * @param max_per_cycle Maximum number of servos to poll in this invocation.
     */
    virtual void refreshHealthSlice(uint8_t max_per_cycle) {}

    /**
     * @brief Get cached blocking/fault state for a joint without performing I/O.
     * @param leg_index Leg index (0..NUM_LEGS-1).
     * @param joint_index Joint index within leg (0..DOF_PER_LEG-1).
     * @return true if the cached state marks this joint as blocked/faulted; false otherwise.
     * @note Default implementation returns false (unknown/no cache).
     */
    virtual bool isBlockedCached(int leg_index, int joint_index) const { return false; }
};

class RobotModel {
  public:
    /**
     * @brief Construct a robot model using the provided parameters.
     * @param params Reference to configuration parameters.
     */
    explicit RobotModel(const Parameters &params);

    /**
     * @brief Destructor - implemented in .cpp to handle unique_ptr properly
     */
    ~RobotModel();

    /**
     * @brief Initialize the WorkspaceAnalyzer with custom configuration.
     * This method allows initializing the WorkspaceAnalyzer after constructing
     * the RobotModel with specific configurations.
     * @param config Compute configuration for the WorkspaceAnalyzer
     * @param validation_config Validation configuration (optional)
     */
    void workspaceAnalyzerInitializer(ComputeConfig config = ComputeConfig::medium(),
                                      const ValidationConfig *validation_config = nullptr);

    /**
     * @brief Get reference to the internal WorkspaceAnalyzer
     * @return Reference to WorkspaceAnalyzer for use by other classes
     */
    WorkspaceAnalyzer &getWorkspaceAnalyzer();

    /**
     * @brief Get const reference to the internal WorkspaceAnalyzer
     * @return Const reference to WorkspaceAnalyzer for read-only access
     */
    const WorkspaceAnalyzer &getWorkspaceAnalyzer() const;

    /**
     * @brief Get default position for a leg based on robot geometry
     * @param leg_index Index of the leg (0-5)
     * @return Default tip position for the leg
     */
    Point3D getLegDefaultPosition(int leg_index) const;

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
    double getTimeDelta() const { return params.time_delta; }

    /** @brief Get joint angle limits in radians (precomputed from degree params). */
    double getCoxaAngleLimitRad(int index) const { return coxa_angle_limits_rad[index]; }
    double getFemurAngleLimitRad(int index) const { return femur_angle_limits_rad[index]; }
    double getTibiaAngleLimitRad(int index) const { return tibia_angle_limits_rad[index]; }

    /** @brief Clamp joint angles to radian limits (OpenSHC parity). */
    void clampToJointLimits(JointAngles &angles) const;

    /**
     * @brief Update the global step frequency (Hz) at runtime.
     * @param step_frequency New step frequency in Hz
     */
    void setStepFrequency(double step_frequency) { params.step_frequency = step_frequency; }

    /**
     * @brief Get the default height offset when all joint angles are 0°
     * @return Default height offset in mm. If default_height_offset is set (non-zero),
     *         returns that value. Otherwise, returns -tibia_length for backwards compatibility.
     */
    double getDefaultHeightOffset() const;

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

    /**
     * @brief Compute horizontal standing reach (coxa pivot to foot projection) for configured standing_height.
     * Uses morphology described in AGENTS.md: tibia assumed vertical in nominal standing pose (femur+tibia angle ≈ 0).
     * Derivation:
     *   target_z = -standing_height = -femur_length * sin(femur_angle) - tibia_length
     *   => sin(femur_angle) = (standing_height - tibia_length) / femur_length
     *   horizontal_projection = femur_length * cos(femur_angle)
     *   standing_horizontal_reach = coxa_length + horizontal_projection
     * Fallback: if standing_height invalid (out of feasible range) returns conservative coxa_length.
     */
    double getStandingHorizontalReach() const;
    // Analytic servo angle computation for target height (tibia vertical assumption)
    static CalculatedServoAngles calculateServoAnglesForHeight(double target_height_mm, const Parameters &params);

    /** Static helper for external code needing the same computation without an instance. */
    static double computeStandingHorizontalReach(const Parameters &p);

    /**
     * @brief Convert GaitType enum to string name
     * @param gait_type GaitType enum value
     * @return String representation of the gait type
     */
    static std::string gaitTypeToString(GaitType gait_type);

    /**
     * @brief Convert string name to GaitType enum
     * @param gait_name String name of the gait
     * @return GaitType enum value (NO_GAIT if not found)
     */
    static GaitType stringToGaitType(const std::string &gait_name);

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
                                const JointAngles &current_angles, double time_delta) const;

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

    // WorkspaceAnalyzer for workspace analysis (OpenSHC-style)
    std::unique_ptr<WorkspaceAnalyzer> workspace_analyzer_;

    JointAngles solveIK(int leg, const Point3D &global_target, JointAngles current,
                        JointAngles current_velocity = JointAngles(0, 0, 0)) const;

    // Helper methods to reduce code duplication
    Point3D transformGlobalToLocalLegCoordinates(int leg, const Point3D &global_target) const;
    void clampJointAngles(JointAngles &angles) const;
};

#endif // ROBOT_MODEL_H
