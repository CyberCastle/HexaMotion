#ifndef LOCOMOTION_SYSTEM_H
#define LOCOMOTION_SYSTEM_H

#include "admittance_controller.h"
#include "body_pose_controller.h"
#include "cartesian_velocity_controller.h"
#include "leg.h"
#include "robot_model.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

#include <vector>
// Forward declarations
class WalkController;

// Main locomotion system class
class LocomotionSystem {
  public:
    // Stop behavior options for stopWalking()
    enum StopMode {
        STOP_UNIFORM, // Force identical stance (phase reset, identity pose) for all legs
        STOP_SOFT     // Force stance without phase reset to preserve continuity
    };

    // Error control
    enum ErrorCode {
        NO_ERROR = 0,
        IMU_ERROR = 1,
        FSR_ERROR = 2,
        SERVO_ERROR = 3,
        KINEMATICS_ERROR = 4,
        STABILITY_ERROR = 5,
        PARAMETER_ERROR = 6,
        SENSOR_ERROR = 7,        // General sensor communication error
        SERVO_BLOCKED_ERROR = 8, // Servo blocked by status flags
        STATE_ERROR = 9          // System state error
    };

  private:
    // Robot parameters
    Parameters params;

    // Hardware interfaces
    IIMUInterface *imu_interface;
    IFSRInterface *fsr_interface;
    IServoInterface *servo_interface;

    // System states
    Eigen::Vector3d body_position;    // Body position [x,y,z]
    Eigen::Vector3d body_orientation; // Body orientation [roll,pitch,yaw]
    Leg legs[NUM_LEGS];               // Leg objects containing all leg data

    // Control variables
    bool system_enabled;

    // Velocity control
    CartesianVelocityController *velocity_controller;

    // Error variables
    ErrorCode last_error;

    RobotModel model;
    BodyPoseController *body_pose_ctrl;
    WalkController *walk_ctrl;
    AdmittanceController *admittance_ctrl;

    // --- Debug / instrumentation helpers ---
    // Track last phase we logged for each leg to debounce repetitive FSR transition spam when other
    // subsystems (e.g., WalkController) overwrite phases each cycle. Only used when debug_fsr_transitions is true.
#ifdef TESTING_ENABLED
    StepPhase last_logged_leg_phase_[NUM_LEGS];
    bool last_logged_initialized_ = false;
#endif

    // System state management
    SystemState system_state;
    bool startup_in_progress;
    bool shutdown_in_progress;

    // Indicates we can resume walking without running the full body startup sequence
    bool resume_from_stop_ = false;

    // Último comando de velocidad deseado (OpenSHC-style persistent velocities)
    double commanded_linear_velocity_x_ = 0.0; // X component
    double commanded_linear_velocity_y_ = 0.0; // Y component
    double commanded_angular_velocity_ = 0.0;

    bool setLegJointAngles(int leg_index, const JointAngles &q);

    // OpenSHC-style IK batch processing functions
    void applyInverseKinematicsToAllLegs();
    void publishJointAnglesToServos();

    // Runtime-only switch to gate coxa servo output during tests
    bool coxa_movement_enabled_ = true;

#ifdef TESTING_ENABLED
    // --- Coxa telemetry instrumentation (TESTING ONLY) ---
    struct CoxaTelemetrySample {
        double time;                         // simulation time (s)
        double global_angle[NUM_LEGS];       // absolute coxa joint angle (rad)
        double local_angle[NUM_LEGS];        // local (offset-compensated) coxa angle (rad)
        double global_velocity[NUM_LEGS];    // d(global_angle)/dt (rad/s)
        double local_velocity[NUM_LEGS];     // d(local_angle)/dt (rad/s)
        double global_accel[NUM_LEGS];       // d(global_velocity)/dt (rad/s^2)
        double local_accel[NUM_LEGS];        // d(local_velocity)/dt (rad/s^2)
        StepPhase phase[NUM_LEGS];           // resolved phase (STANCE/SWING)
        double stride_dx[NUM_LEGS];          // current (tip.x - stance_start_tip.x) in global frame (mm)
        double stride_dy[NUM_LEGS];          // current (tip.y - stance_start_tip.y) in global frame (mm)
        double body_vel_x;                   // commanded linear X velocity (mm/s)
        double body_vel_y;                   // commanded linear Y velocity (mm/s)
        double body_ang_vel;                 // commanded angular velocity (deg/s)
        double servo_command_coxa[NUM_LEGS]; // last servo command for coxa (rad, internal sign-compensated)
    };
    bool telemetry_enabled_ = false;                      // runtime toggle
    double telemetry_time_accumulator_ = 0.0;             // simulated time accumulator
    std::vector<CoxaTelemetrySample> telemetry_;          // ring / linear buffer
    static constexpr size_t kMaxTelemetrySamples_ = 8192; // cap to avoid unbounded growth in tests
    // Previous state for velocity/accel estimation
    double prev_coxa_angle_[NUM_LEGS] = {0};
    double prev_coxa_velocity_[NUM_LEGS] = {0};
    bool prev_valid_ = false;
    // Stride start tip position per leg (updated when entering STANCE)
    Point3D stride_start_tip_[NUM_LEGS];
    bool stride_start_valid_[NUM_LEGS] = {false, false, false, false, false, false};
    // Last servo command (degrees) captured in publish step to compare vs internal angle
    double last_servo_cmd_deg_[NUM_LEGS][DOF_PER_LEG] = {{0}};

    void recordCoxaTelemetrySample(); // internal helper (called inside update())
  public:                             // test-only public accessors (still under TESTING_ENABLED)
    void enableTelemetry(bool enable) { telemetry_enabled_ = enable; }
    bool isTelemetryEnabled() const { return telemetry_enabled_; }
    size_t getTelemetrySampleCount() const { return telemetry_.size(); }
    const CoxaTelemetrySample &getTelemetrySample(size_t idx) const { return telemetry_[idx]; }
#endif

  public:
    /**
     * @brief Construct a locomotion system with the given parameters.
     * @param params Physical and control parameters for the robot.
     */
    explicit LocomotionSystem(const Parameters &params);

    /**
     * @brief Destructor releases allocated controllers.
     */
    ~LocomotionSystem();

    /**
     * @brief Initialize hardware interfaces.
     *
     * @param imu   IMU interface implementation.
     * @param fsr   FSR interface implementation.
     * @param servo Servo interface implementation.
     * @param pose_config Pose configuration for the robot.
     * @return True on successful initialization.
     */
    bool initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo, const BodyPoseConfiguration &body_pose_config);

    /**
     * @brief Run calibration sequence for sensors and servos.
     * @return True if calibration succeeds.
     */
    bool calibrateSystem();

    /**
     * @brief Check if the locomotion system is enabled.
     * @return True when the system is running.
     */
    bool isSystemEnabled() const;

    // Inverse kinematics
    /** Compute joint angles for a desired leg tip position. */
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);

    /** Check if target is within workspace. */
    bool isTargetReachable(int leg_index, const Point3D &target);
    /** Project target to workspace boundary if outside. */
    Point3D constrainToWorkspace(int leg_index, const Point3D &target);
    /** Get joint limit proximity (1.0 = far from limits, 0.0 = at limits). */
    double getJointLimitProximity(int leg_index, const JointAngles &angles);

    // Gait planner
    /** Select the active gait type. */
    bool setGaitType(GaitType gait);
    /** Plan the next gait step from desired velocities. */
    bool planGaitSequence(double velocity_x, double velocity_y, double angular_velocity);

    // State management (OpenSHC equivalent)
    /** Check if startup sequence is in progress */
    bool isStartupInProgress() const { return startup_in_progress; }
    /** Check if shutdown sequence is in progress */
    bool isShutdownInProgress() const { return shutdown_in_progress; }
    /** Get current system state */
    SystemState getSystemState() const { return system_state; }
    /** Get startup progress percent (0-100). Returns 100 if startup already completed or controller missing. */
    int getStartupProgressPercent() const {
        if (!body_pose_ctrl)
            return 100;
        if (!startup_in_progress)
            return 100;
        return body_pose_ctrl->getStartupProgressPercent();
    }

    // Locomotion control
    /** Start walking forward indefinitely. */
    bool walkForward(double velocity);
    /** Start walking backward indefinitely. */
    bool walkBackward(double velocity);
    /** Rotate the robot in place indefinitely. */
    bool turnInPlace(double angular_velocity);
    /** Walk laterally to one side indefinitely. */
    bool walkSideways(double velocity, bool right_direction = true);

    /**
     * @brief Execute one iteration of the startup sequence.
     * Wraps BodyPoseController::executeStartupSequence and handles transition to RUNNING state.
     */
    bool executeStartupSequence();

    /**
     * @brief Execute one iteration of the shutdown sequence.
     * Wraps BodyPoseController::executeShutdownSequence and handles transition to READY state.
     */
    bool executeShutdownSequence();

    // OpenSHC-style walking control
    /** Start walking (startup sequence only). Gait and velocities must have been set beforehand. */
    bool startWalking();

    /** Stop walking and keep all feet on ground without shutdown; behavior selectable */
    bool stopWalking(StopMode mode = STOP_UNIFORM);

    // Stability analysis
    /** Verify that current pose maintains stability margin. */
    bool checkStabilityMargin();
    /** Calculate center of pressure under the robot. */
    Eigen::Vector2d calculateCenterOfPressure();
    /** Compute a numeric stability index. */
    double calculateStabilityIndex();
    /** Enhanced stability calculation using absolute positioning data. */
    double calculateDynamicStabilityIndex();
    /** Check if the robot is statically stable. */
    bool isStaticallyStable();

    // Body pose control
    /** Set robot to standing pose */
    bool setStandingPose();

    /**
     * @brief Begin non-blocking jerk-limited transition to standing pose (profiles created in BodyPoseController).
     * @return true if started or already complete.
     */
    bool establishInitialStandingPose();

    /** Advance one iteration of the initial standing pose transition; sends servo commands for current S-curve sample. */
    bool stepInitialStandingPose();

    /** Query if initial standing pose transition is active. */
    bool isInitialStandingPoseActive() const { return body_pose_ctrl && body_pose_ctrl->isInitialStandingPoseActive(); }

    /** Set body pose with position and orientation */
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /** Check if smooth movement is in progress */
    bool isSmoothMovementInProgress() const;

    /** Reset smooth movement trajectory */
    void resetSmoothMovement();

    ErrorCode getLastError() const { return last_error; }
    String getErrorMessage(ErrorCode error);
    bool handleError(ErrorCode error);

    // Leg state management
    /** Update leg contact states based on FSR sensor readings. */
    void updateLegStates();

    // System update
    /** Update all controllers and state machines. */
    bool update();

    /** Update FSR and IMU sensors in parallel for optimal performance. */
    bool updateSensorsParallel();

    /** Enable/disable coxa joint movement globally */
    void setCoxaMovementEnabled(bool enabled) { coxa_movement_enabled_ = enabled; }

    /** Query coxa joint movement state. */
    bool isCoxaMovementEnabled() const { return coxa_movement_enabled_; }

    // Getters
    const Parameters &getParameters() const { return params; }
    RobotModel &getRobotModel() { return model; }
    const RobotModel &getRobotModel() const { return model; }
    IServoInterface *getServoInterface() { return servo_interface; }
    Eigen::Vector3d getBodyPosition() const { return body_position; }
    Eigen::Vector3d getBodyOrientation() const { return body_orientation; }
    StepPhase getLegState(int leg_index) const { return legs[leg_index].getStepPhase(); }
    JointAngles getJointAngles(int leg_index) const { return legs[leg_index].getJointAngles(); }
    Point3D getLegPosition(int leg_index) const { return legs[leg_index].getCurrentTipPositionGlobal(); }

    // Leg access methods
    /** Get leg object by index. */
    const Leg &getLeg(int leg_index) const { return legs[leg_index]; }
    /** Get leg object by index (mutable). */
    Leg &getLeg(int leg_index) { return legs[leg_index]; }

    // Setters
    /** Replace the current parameter set. */
    bool setParameters(const Parameters &new_params);

    // Cartesian velocity control
    /** Get the velocity controller instance for configuration. */
    CartesianVelocityController *getVelocityController() { return velocity_controller; }

    /** Enable or disable velocity-based servo speed control. */
    bool setVelocityControlEnabled(bool enable);

    /** Configure velocity scaling parameters. */
    bool setVelocityScaling(const CartesianVelocityController::VelocityScaling &scaling);

    /** Configure gait-specific speed modifiers. */
    bool setGaitSpeedModifiers(const CartesianVelocityController::GaitSpeedModifiers &modifiers);

    /** Get current servo speed for a specific joint (affected by velocity control). */
    double getCurrentServoSpeed(int leg_index, int joint_index) const;

    // Getter for WalkController
    WalkController *getWalkController() { return walk_ctrl; }

    /** Direct access to BodyPoseController (tests & advanced instrumentation) */
    BodyPoseController *getBodyPoseController() { return body_pose_ctrl; }
    const BodyPoseController *getBodyPoseController() const { return body_pose_ctrl; }

  private:
    // Helper methods
    double constrainAngle(double angle, double min_angle, double max_angle);
    bool validateParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);

    // (Removed unused adaptive control helpers flagged by static analysis)
};

#include "math_utils.h"

#endif // LOCOMOTION_SYSTEM_H
