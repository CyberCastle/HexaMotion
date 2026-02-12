#ifndef LOCOMOTION_SYSTEM_H
#define LOCOMOTION_SYSTEM_H

#include "admittance_controller.h"
#include "body_pose_controller.h"
#include "cartesian_velocity_controller.h"
#include "leg.h"
#include "robot_model.h"
#include "state_controller_context.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

#include <memory>
#include <string>
#include <vector>
/** Forward declarations. */
class WalkController;
class StateController;

/** Main locomotion system class. */
class LocomotionSystem : public StateControllerContext {
  public:
    /** Stop behavior options for stopWalking(). */
    enum StopMode {
        STOP_UNIFORM, /**< Force identical stance (phase reset, identity pose) for all legs. */
        STOP_SOFT     /**< Force stance without phase reset to preserve continuity. */
    };

    /** Maximum time (seconds) to wait for initial servo joint readback during initialization. */
    /** Equivalent to OpenSHC ACQUISTION_TIME in main.cpp. */
    static constexpr int ACQUISITION_TIMEOUT_S = 10;

    /** Error control. */
    enum ErrorCode {
        NO_ERROR = 0,
        IMU_ERROR = 1,
        FSR_ERROR = 2,
        SERVO_ERROR = 3,
        KINEMATICS_ERROR = 4,
        STABILITY_ERROR = 5,
        PARAMETER_ERROR = 6,
        SENSOR_ERROR = 7,        /**< General sensor communication error. */
        SERVO_BLOCKED_ERROR = 8, /**< Servo blocked by status flags. */
        STATE_ERROR = 9          /**< System state error. */
    };

  private:
    /** Robot parameters. */
    Parameters params;

    /** Hardware interfaces. */
    IIMUInterface *imu_interface;
    IFSRInterface *fsr_interface;
    IServoInterface *servo_interface;

    /** System states. */
    Eigen::Vector3d body_position;    /**< Body position [x,y,z]. */
    Eigen::Vector3d body_orientation; /**< Body orientation [roll,pitch,yaw]. */
    Leg legs[NUM_LEGS];               /**< Leg objects containing all leg data. */

    /** Control variables. */
    bool system_enabled;

    /** Velocity control. */
    CartesianVelocityController *velocity_controller;

    /** Error variables. */
    ErrorCode last_error;

    RobotModel model;
    BodyPoseController *body_pose_ctrl;
    WalkController *walk_ctrl;
    AdmittanceController *admittance_ctrl;

    /** State controller (owns the orchestration state machine; created in initialize()). */
    std::unique_ptr<StateController> state_controller_;

    /** Debug/instrumentation helpers. */
    /**
     * @brief Track last phase logged per leg to debounce repetitive FSR transition spam.
     *
     * Used only when debug_fsr_transitions is true and other subsystems overwrite phases each cycle.
     */
#ifdef TESTING_ENABLED
    StepPhase last_logged_leg_phase_[NUM_LEGS];
    bool last_logged_initialized_ = false;
#endif

    /** System state management. */
    SystemState system_state;
    bool startup_in_progress;
    bool shutdown_in_progress;

    /** Joint acquisition state (OpenSHC ACQUISTION_TIME equivalent). */
    bool joint_positions_initialised_ = false;

    /**
     * @brief Attempt to read initial joint positions from all servos within a timeout.
     *
     * Polls servo_interface->getJointAngle() for every joint and considers acquisition
     * successful when all joints return finite values. Equivalent to the spin-wait loop
     * in OpenSHC main.cpp that waits up to ACQUISTION_TIME for joint callbacks.
     *
     * @return true if all joints responded with valid angles before the timeout.
     */
    bool attemptJointAcquisition();

    /** Last desired velocity command (OpenSHC-style persistent velocities). */
    double commanded_linear_velocity_x_ = 0.0; /**< X component. */
    double commanded_linear_velocity_y_ = 0.0; /**< Y component. */
    double commanded_angular_velocity_ = 0.0;

    bool setLegJointAngles(int leg_index, const JointAngles &q);

    /** OpenSHC-style IK batch processing functions. */
    void applyInverseKinematicsToAllLegs();
    void publishJointAnglesToServos();

    /** Update per-joint telemetry (velocity/effort) from servo interface if available. */
    void updateLegJointTelemetry();

    /** Runtime-only switch to gate coxa servo output during tests. */
    bool coxa_movement_enabled_ = true;

#ifdef TESTING_ENABLED
    /** Coxa telemetry instrumentation (testing only). */
    struct CoxaTelemetrySample {
        double time;                         /**< Simulation time (s). */
        double global_angle[NUM_LEGS];       /**< Absolute coxa joint angle (rad). */
        double local_angle[NUM_LEGS];        /**< Local (offset-compensated) coxa angle (rad). */
        double global_velocity[NUM_LEGS];    /**< d(global_angle)/dt (rad/s). */
        double local_velocity[NUM_LEGS];     /**< d(local_angle)/dt (rad/s). */
        double global_accel[NUM_LEGS];       /**< d(global_velocity)/dt (rad/s^2). */
        double local_accel[NUM_LEGS];        /**< d(local_velocity)/dt (rad/s^2). */
        StepPhase phase[NUM_LEGS];           /**< Resolved phase (STANCE/SWING). */
        double stride_dx[NUM_LEGS];          /**< tip.x - stance_start_tip.x in global frame (mm). */
        double stride_dy[NUM_LEGS];          /**< tip.y - stance_start_tip.y in global frame (mm). */
        double body_vel_x;                   /**< Commanded linear X velocity (mm/s). */
        double body_vel_y;                   /**< Commanded linear Y velocity (mm/s). */
        double body_ang_vel;                 /**< Commanded angular velocity (deg/s). */
        double servo_command_coxa[NUM_LEGS]; /**< Last servo command for coxa (rad, sign-compensated). */
    };
    bool telemetry_enabled_ = false;                      /**< Runtime toggle. */
    double telemetry_time_accumulator_ = 0.0;             /**< Simulated time accumulator. */
    std::vector<CoxaTelemetrySample> telemetry_;          /**< Ring/linear buffer. */
    static constexpr size_t kMaxTelemetrySamples_ = 8192; /**< Cap to avoid unbounded growth in tests. */
    /** Previous state for velocity/accel estimation. */
    double prev_coxa_angle_[NUM_LEGS] = {0};
    double prev_coxa_velocity_[NUM_LEGS] = {0};
    bool prev_valid_ = false;
    /** Stride start tip position per leg (updated when entering STANCE). */
    Point3D stride_start_tip_[NUM_LEGS];
    bool stride_start_valid_[NUM_LEGS] = {false, false, false, false, false, false};
    /** Last servo command (degrees) captured in publish step to compare vs internal angle. */
    double last_servo_cmd_deg_[NUM_LEGS][DOF_PER_LEG] = {{0}};

    /** Internal helper (called inside update()). */
    void recordCoxaTelemetrySample();

  public: /**< Test-only public accessors (still under TESTING_ENABLED). */
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
    bool isSystemEnabled() const override;

    /** Inverse kinematics. */
    /** Compute joint angles for a desired leg tip position. */
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);

    /** Check if target is within workspace. */
    bool isTargetReachable(int leg_index, const Point3D &target);
    /** Project target to workspace boundary if outside. */
    Point3D constrainToWorkspace(int leg_index, const Point3D &target);
    /** Get joint limit proximity (1.0 = far from limits, 0.0 = at limits). */
    double getJointLimitProximity(int leg_index, const JointAngles &angles);

    /** Gait planner. */
    /** Select the active gait configuration. */
    bool setGaitConfiguration(const GaitConfiguration &gait_config) override;
    /** Plan the next gait step from desired velocities. */
    bool planGaitSequence(double velocity_x, double velocity_y, double angular_velocity) override;

    /**
     * @brief Update a runtime-adjustable parameter by name (lightweight parity with OpenSHC).
     * @param name Parameter name (step_frequency, swing_height, stance_span_modifier)
     * @param value New parameter value
     * @return True if parameter was accepted and applied
     */
    bool setParameter(const std::string &name, double value);

    /** State management (OpenSHC equivalent). */
    /** Check if startup sequence is in progress. */
    bool isStartupInProgress() const { return startup_in_progress; }
    /** Check if shutdown sequence is in progress. */
    bool isShutdownInProgress() const { return shutdown_in_progress; }
    /**
     * @brief Check if initial joint positions were successfully acquired from servos.
     *
     * Equivalent to OpenSHC StateController::jointPositionsInitialised().
     */
    bool jointPositionsInitialised() const { return joint_positions_initialised_; }
    /**
     * @brief Get the internal StateController.
     * @return Pointer to StateController (non-null after initialize()).
     */
    StateController *getStateController() const { return state_controller_.get(); }
    /** Get current system state. */
    SystemState getSystemState() const override { return system_state; }
    /** Get current composed body pose (OpenSHC Model::getCurrentPose equivalent). */
    Pose getCurrentBodyPose() const;
    /** Check if legs are bearing load based on body pose controller estimate. */
    bool legsBearingLoad() const;
    /** Get startup progress percent (0-100). Returns 100 if startup is complete or controller missing. */
    int getStartupProgressPercent() const override {
        if (!body_pose_ctrl)
            return 100;
        if (!startup_in_progress)
            return 100;
        return body_pose_ctrl->getStartupProgressPercent();
    }

    /** Locomotion control. */
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
    bool executeStartupSequence() override;

    /**
     * @brief Execute one iteration of the shutdown sequence.
     * Wraps BodyPoseController::executeShutdownSequence and handles transition to READY state.
     */
    bool executeShutdownSequence() override;

    /** OpenSHC-style walking control. */
    /** Start walking (startup sequence only). Gait and velocities must have been set beforehand. */
    bool startWalking();

    /** Stop walking and keep all feet on ground without shutdown; behavior selectable. */
    bool stopWalking(StopMode mode = STOP_UNIFORM);

    /** Stability analysis. */
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

    /** Body pose control. */
    /** Set robot to standing pose. */
    bool setStandingPose() override;

    /**
     * @brief Begin non-blocking jerk-limited transition to standing pose (profiles created in BodyPoseController).
     * @return true if started or already complete.
     */
    bool establishInitialStandingPose() override;

    /** Advance one iteration of the initial standing pose transition; sends servo commands for current S-curve sample. */
    bool stepInitialStandingPose();

    /** Query if initial standing pose transition is active. */
    bool isInitialStandingPoseActive() const override { return body_pose_ctrl && body_pose_ctrl->isInitialStandingPoseActive(); }

    /** Set body pose with position and orientation (orientation in radians). */
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /**
     * @brief Set manual body pose input for walking pose composition.
     * @param position Body translation (mm)
     * @param orientation Body rotation (roll,pitch,yaw in radians)
     * @return True if accepted
     */
    bool setManualBodyPoseInput(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation) override;

    bool applyManualLegInputs(int primary_leg_index,
                              const Eigen::Vector3d &primary_tip_velocity,
                              int secondary_leg_index,
                              const Eigen::Vector3d &secondary_tip_velocity,
                              bool primary_pose_valid,
                              const Point3D &primary_tip_pose,
                              bool secondary_pose_valid,
                              const Point3D &secondary_tip_pose) override;

    /** Check if smooth movement is in progress. */
    bool isSmoothMovementInProgress() const;

    /** Reset smooth movement trajectory. */
    void resetSmoothMovement();

    ErrorCode getLastError() const { return last_error; }
    String getErrorMessage(ErrorCode error);
    bool handleError(ErrorCode error);

    /** Leg state management. */
    /** Update leg contact states based on FSR sensor readings. */
    void updateLegStates();

    /** System update. */
    /** Update all controllers and state machines. */
    bool update();

    /** Execute one low-level locomotion pipeline iteration (context hook for StateController). */
    bool runControlPipelineStep() override;

    /** Update FSR and IMU sensors in parallel for optimal performance. */
    bool updateSensorsParallel();

    /** Enable/disable coxa joint movement globally. */
    void setCoxaMovementEnabled(bool enabled) { coxa_movement_enabled_ = enabled; }

    /** Query coxa joint movement state. */
    bool isCoxaMovementEnabled() const { return coxa_movement_enabled_; }

    /** Getters. */
    const Parameters &getParameters() const { return params; }
    RobotModel &getRobotModel() override { return model; }
    const RobotModel &getRobotModel() const { return model; }
    IServoInterface *getServoInterface() { return servo_interface; }
    Eigen::Vector3d getBodyPosition() const override { return body_position; }
    Eigen::Vector3d getBodyOrientation() const override { return body_orientation; }
    StepPhase getLegState(int leg_index) const { return legs[leg_index].getStepPhase(); }
    JointAngles getJointAngles(int leg_index) const override { return legs[leg_index].getJointAngles(); }
    Point3D getLegPosition(int leg_index) const { return legs[leg_index].getCurrentTipPositionGlobal(); }

    /** Leg access methods. */
    /** Get leg object by index. */
    const Leg &getLeg(int leg_index) const { return legs[leg_index]; }
    /** Get leg object by index (mutable). */
    Leg &getLeg(int leg_index) { return legs[leg_index]; }
    /** Get pointer to legs array (for batch operations like poseForLegManipulation). */
    Leg *getLegsArray() override { return legs; }

    /** Setters. */
    /** Replace the current parameter set. */
    bool setParameters(const Parameters &new_params);

    /** Cartesian velocity control. */
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

    /** Get robot parameters. */
    const Parameters &getParams() const override { return params; }

    /** Getter for WalkController. */
    WalkController *getWalkController() override { return walk_ctrl; }

    /** Direct access to BodyPoseController (tests and advanced instrumentation). */
    BodyPoseController *getBodyPoseController() override { return body_pose_ctrl; }
    const BodyPoseController *getBodyPoseController() const { return body_pose_ctrl; }

  private:
    /** Helper methods. */
    double constrainAngle(double angle, double min_angle, double max_angle);
    bool validateParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);

    /** Removed unused adaptive control helpers flagged by static analysis. */
};

#include "math_utils.h"

#endif /**< LOCOMOTION_SYSTEM_H */
