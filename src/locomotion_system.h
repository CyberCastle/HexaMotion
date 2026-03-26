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
    /**
     * @brief OpenSHC-like desired command replica for one physical joint.
     *
     * This structure is intentionally exposed so an external orchestrator can mirror
     * OpenSHC semantics around desired_* and prev_desired_* without ROS transport.
     *
     * Units:
     * - desired_position_rad / prev_desired_position_rad: radians
     * - desired_velocity_rad_s / prev_desired_velocity_rad_s: rad/s
     * - desired_effort / prev_desired_effort: driver units (implementation-defined)
     */
    /**
     * @brief Runtime desired-command state per physical joint.
     *
     * This mirrors OpenSHC joint fields used by external orchestration logic:
     * desired_position_, desired_velocity_, desired_effort_ and their prev_desired_* counterparts.
     */
    struct DesiredJointCommandState {
        double desired_position_rad = 0.0;
        double desired_velocity_rad_s = 0.0;
        double desired_effort = 0.0;
        double prev_desired_position_rad = 0.0;
        double prev_desired_velocity_rad_s = 0.0;
        double prev_desired_effort = 0.0;
    };

    /**
     * @brief Velocity command snapshot (OpenSHC shc/velocity publisher equivalent).
     */
    struct VelocityCommand {
        double linear_x;  /**< Forward velocity (mm/s). */
        double linear_y;  /**< Lateral velocity (mm/s). */
        double angular_z; /**< Rotational velocity (deg/s). */
    };

    /**
     * @brief Body pose snapshot (OpenSHC shc/pose publisher equivalent).
     *
     * Position in mm, orientation in radians.
     */
    struct BodyPoseCommand {
        double position_x; /**< X translation (mm). */
        double position_y; /**< Y translation (mm). */
        double position_z; /**< Z translation (mm). */
        double roll;       /**< Roll (rad). */
        double pitch;      /**< Pitch (rad). */
        double yaw;        /**< Yaw (rad). */
    };

    /**
     * @brief Walkspace radius data (OpenSHC shc/walkspace publisher equivalent).
     */
    struct WalkspaceInfo {
        double average_radius; /**< Average walkspace radius (mm). */
        double min_radius;     /**< Minimum walkspace radius (mm). */
        double max_radius;     /**< Maximum walkspace radius (mm). */
    };

    /**
     * @brief IMU rotation pose error (OpenSHC shc/rotation_pose_error publisher equivalent).
     *
     * Contains PID error terms used by IMU-based body pose compensation.
     */
    struct RotationPoseError {
        Eigen::Vector3d absement_error; /**< Integral of position error. */
        Eigen::Vector3d position_error; /**< Current rotation error. */
        Eigen::Vector3d velocity_error; /**< Derivative of rotation error. */
    };

    /**
     * @brief Comprehensive per-leg state (OpenSHC shc/{leg_id}/state publisher equivalent).
     *
     * Aggregates tip poses from each pipeline stage, joint state, gait progress,
     * and force/admittance data for a single leg.
     */
    struct LegStateInfo {
        Point3D walker_tip_pose;          /**< Tip from LegStepper trajectory. */
        Point3D target_tip_pose;          /**< Desired tip position. */
        Point3D model_tip_pose;           /**< Tip after IK (model position). */
        JointAngles joint_positions;      /**< Current joint angles (rad). */
        JointAngles joint_velocities;     /**< Current joint velocities (rad/s). */
        JointAngles joint_efforts;        /**< Current joint efforts (driver units). */
        double step_progress;             /**< Phase progress (0-1). */
        StepPhase step_phase;             /**< Current step phase. */
        LegState leg_state;               /**< Leg state (WALKING/MANUAL/transitioning). */
        double contact_force;             /**< Contact force magnitude. */
        Eigen::Vector3d admittance_delta; /**< Compliance offset (mm). */
        double virtual_stiffness;         /**< Dynamically scaled stiffness. */
        Eigen::Vector3d tip_force;        /**< Calculated tip force. */
        Point3D stride_vector;            /**< Current stride vector. */
        Point3D tip_velocity;             /**< Tip velocity (mm/s). */
    };

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
    /** Robot kinematic model — must be declared before legs so that it is fully
     *  constructed when the Leg initialisers reference it (C++ §15.6.2). */
    RobotModel model;

    Leg legs[NUM_LEGS]; /**< Leg objects containing all leg data. */

    /** Control variables. */
    bool system_enabled;

    /** Velocity control. */
    CartesianVelocityController *velocity_controller;

    /** Error variables. */
    ErrorCode last_error;
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

    /** Initial standing pose transition state. */
    bool initial_standing_active_ = false;

    /** Last reported startup progress (tracked in LS, not BPC). */
    int last_startup_progress_ = 0;

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

    /** OpenSHC-style primary/secondary leg selection (syropod_remote/{primary,secondary}_leg_selection). */
    int primary_leg_selection_ = -1;
    int secondary_leg_selection_ = -1;

    /** Cached manual leg inputs to enforce OpenSHC update order: walk first, manual second. */
    int pending_primary_leg_index_ = -1;
    int pending_secondary_leg_index_ = -1;
    Eigen::Vector3d pending_primary_tip_velocity_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d pending_secondary_tip_velocity_ = Eigen::Vector3d::Zero();
    bool pending_primary_pose_valid_ = false;
    bool pending_secondary_pose_valid_ = false;
    Point3D pending_primary_tip_pose_;
    Point3D pending_secondary_tip_pose_;

    bool step_frequency_adjust_pending_ = false;
    double pending_step_frequency_ = 0.0;

    bool setLegJointAngles(int leg_index, const JointAngles &q);

    /** Apply configured per-joint output offset in degrees. */
    double applyJointOutputCalibration(int leg_index, int joint_index, double commanded_angle_deg) const;

    /** Limit per-joint commanded angle slew rate using configured deg/s cap. */
    double limitJointAngularSpeedCommand(int leg_index, int joint_index, double target_angle_deg);

    /** OpenSHC-style IK batch processing functions. */
    void applyInverseKinematicsToAllLegs();
    void publishJointAnglesToServos();

    /** Update per-joint telemetry (velocity/effort) from servo interface if available. */
    void updateLegJointTelemetry();

    /** Runtime-only switch to gate coxa servo output during tests. */
    bool coxa_movement_enabled_ = true;

    /** Last commanded servo angles (degrees) for per-joint slew limiting. */
    double last_joint_command_deg_[NUM_LEGS][DOF_PER_LEG] = {{0.0}};
    bool last_joint_command_valid_[NUM_LEGS][DOF_PER_LEG] = {{false}};

    /**
     * Desired/previous desired command state per joint.
     *
     * Indexing:
     * - First dimension: leg index [0..NUM_LEGS-1]
     * - Second dimension: joint index [0..DOF_PER_LEG-1] => 0=coxa, 1=femur, 2=tibia
     */
    DesiredJointCommandState desired_joint_command_state_[NUM_LEGS][DOF_PER_LEG];

    bool isValidJointAddress(int leg_index, int joint_index) const;
    void syncDesiredJointStateFromInternalCommand(int leg_index, const JointAngles &desired_positions,
                                                  const JointAngles &desired_velocities);
    bool canApplyStepFrequency(double step_frequency) const;
    bool applyStepFrequency(double step_frequency);
    bool tryApplyPendingStepFrequency();

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
     * @brief Set gait step frequency with velocity-guarded transition.
     * @param value New step frequency (Hz).
     * @return True if accepted (applied now or deferred until safe).
     */
    bool setStepFrequency(double value);

    /**
     * @brief Set gait swing height.
     * @param value New swing height (mm).
     * @return True if applied.
     */
    bool setSwingHeight(double value);

    /**
     * @brief Set gait stance span modifier.
     * @param value New stance span modifier.
     * @return True if applied.
     */
    bool setStanceSpanModifier(double value);

    /**
     * @brief Set gait swing width.
     * @param value New swing width (mm).
     * @return True if applied.
     */
    bool setSwingWidth(double value);

    /**
     * @brief Set gait step depth.
     * @param value New step depth (mm).
     * @return True if applied.
     */
    bool setStepDepth(double value);

    /**
     * @brief Set admittance virtual mass.
     * @param value New virtual mass.
     * @return True if applied.
     */
    bool setVirtualMass(double value);

    /**
     * @brief Set admittance virtual stiffness.
     * @param value New virtual stiffness.
     * @return True if applied.
     */
    bool setVirtualStiffness(double value);

    /**
     * @brief Set admittance virtual damping ratio.
     * @param value New virtual damping ratio.
     * @return True if applied.
     */
    bool setVirtualDamping(double value);

    /**
     * @brief Set admittance force gain.
     * @param value New force gain.
     * @return True if applied.
     */
    bool setForceGain(double value);

    /** State management (OpenSHC equivalent). */
    /**
     * @brief Persist current achieved leg configuration as the new default reference.
     *
     * OpenSHC equivalent of Model::updateDefaultConfiguration(). This updates
     * per-leg defaults and refreshes walkspace/workspace references so subsequent
     * reachability constraints remain consistent with the new stance baseline.
     *
     * @return True if defaults were updated successfully.
     */
    bool updateDefaultConfiguration();

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
    /** Get current robot state from the internal StateController. */
    RobotState getRobotState() const;
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
        return last_startup_progress_;
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

    /** Body pose control. */
    /** Set robot to standing pose. */
    bool setStandingPose() override;

    /** Apply full configured joint pose targets for all legs. */
    bool setRobotJointAngles(const JointAngles target_angles[NUM_LEGS]) override;

    /**
     * @brief Begin non-blocking transition to standing pose (profiles created in BodyPoseController).
     * @return true if started or already complete.
     */
    bool establishInitialStandingPose() override;

    /** Advance one iteration of the initial standing pose transition; sends servo commands for the current transition sample. */
    bool stepInitialStandingPose();

    /** Query if initial standing pose transition is active. */
    bool isInitialStandingPoseActive() const override { return initial_standing_active_; }

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

    /**
     * @brief Update admittance stiffness for a single leg during state transition.
     * OpenSHC equivalent: admittance_->updateStiffness(leg, scale_reference) in legStateToggle().
     */
    void updateAdmittanceStiffness(int leg_index, double scale_reference) override;

    /**
     * @brief Activate RUNNING state directly (no startup sequence).
     * Initializes walk controller, leg phases, and sets system_state = OPERATIONAL.
     */
    bool activateRunningState() override;

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

    /**
     * @brief Start a new external desired-command cycle by shifting desired_* to prev_desired_*.
     *
     * External software can call this once per control cycle before writing new desired
     * values to replicate OpenSHC's prev_desired_* progression semantics.
     *
     * Recommended external flow per cycle:
     * 1) call beginDesiredJointCommandCycle()
     * 2) call setDesiredJointCommandState()/setDesiredJointEffort() as needed
     * 3) optionally read back via getDesiredJointCommandState()
     *
     * Note: the internal locomotion pipeline also updates this state from commanded
     * joint targets, so external software can use this as a unified observability point.
     */
    void beginDesiredJointCommandCycle();

    /**
     * @brief Set desired position/velocity/effort for one joint from external software.
     * @param leg_index Leg index (0..NUM_LEGS-1)
     * @param joint_index Joint index (0=coxa,1=femur,2=tibia)
     * @param desired_position_rad Desired joint position in radians
     * @param desired_velocity_rad_s Desired joint velocity in rad/s
     * @param desired_effort Desired joint effort in driver units
     * @return True when indices are valid and state was updated
     */
    bool setDesiredJointCommandState(int leg_index, int joint_index,
                                     double desired_position_rad,
                                     double desired_velocity_rad_s,
                                     double desired_effort);

    /**
     * @brief Update only desired effort for one joint from external software.
     * @param leg_index Leg index (0..NUM_LEGS-1)
     * @param joint_index Joint index (0=coxa,1=femur,2=tibia)
     * @param desired_effort Desired joint effort in driver units
     * @return True when indices are valid and state was updated
     */
    bool setDesiredJointEffort(int leg_index, int joint_index, double desired_effort);

    /**
     * @brief Get desired/previous desired command state for one joint.
     * @param leg_index Leg index (0..NUM_LEGS-1)
     * @param joint_index Joint index (0=coxa,1=femur,2=tibia)
     * @param state Output command state
     * @return True when indices are valid and state was written
     */
    bool getDesiredJointCommandState(int leg_index, int joint_index, DesiredJointCommandState &state) const;

    /** Get robot parameters. */
    const Parameters &getParams() const override { return params; }

    /** Getter for WalkController. */
    WalkController *getWalkController() override { return walk_ctrl; }

    /** Direct access to BodyPoseController (tests and advanced instrumentation). */
    BodyPoseController *getBodyPoseController() override { return body_pose_ctrl; }
    const BodyPoseController *getBodyPoseController() const { return body_pose_ctrl; }

    /** @name ROS-equivalent subscription accessors (input setters).
     *
     *  These methods mirror OpenSHC ROS subscriber callbacks, translating
     *  external commands into the internal control pipeline without ROS
     *  transport.  Each setter is named after the conceptual OpenSHC topic
     *  it replaces (see AGENTS.md).
     *  @{ */

    /**
     * @brief Set system state (OpenSHC syropod_remote/system_state subscriber equivalent).
     * @param state Desired system state.
     * @return True if the state transition was accepted.
     */
    bool setSystemState(SystemState state);

    /**
     * @brief Set robot state (OpenSHC syropod_remote/robot_state subscriber equivalent).
     * @param state Desired robot state.
     * @return True if the state transition was accepted.
     */
    bool setRobotState(RobotState state);

    /**
     * @brief Set desired body velocity (OpenSHC syropod_remote/desired_velocity subscriber equivalent).
     *
     * Stores input velocity in StateController; body_velocity_scaler is applied
     * during the internal pipeline (planGaitSequence), matching OpenSHC semantics.
     *
     * @param linear_x  Forward velocity (mm/s).
     * @param linear_y  Lateral velocity (mm/s).
     * @param angular_z Rotational velocity (deg/s).
     */
    void setDesiredVelocity(double linear_x, double linear_y, double angular_z);

    /**
     * @brief Set desired body pose (OpenSHC syropod_remote/desired_pose subscriber equivalent).
     *
     * Position in mm, orientation in radians.
     *
     * @param position    Body translation (mm).
     * @param orientation Body rotation (roll, pitch, yaw in radians).
     */
    void setDesiredPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /**
     * @brief Set posing mode (OpenSHC syropod_remote/posing_mode subscriber equivalent).
     * @param mode Desired posing mode.
     * @return True if mode change was accepted.
     */
    bool setPosingMode(PosingMode mode);

    /**
     * @brief Set pose reset mode (OpenSHC syropod_remote/pose_reset_mode subscriber equivalent).
     * @param mode Desired reset mode.
     * @return True if mode change was accepted.
     */
    bool setPoseResetMode(PoseResetMode mode);

    /**
     * @brief Select active gait (OpenSHC syropod_remote/gait_selection subscriber equivalent).
     * @param gait Desired gait type.
     * @return True if gait change was accepted.
     */
    bool selectGait(GaitType gait);

    /**
     * @brief Select the primary leg for manual control (OpenSHC syropod_remote/primary_leg_selection subscriber equivalent).
     * @param leg_index Leg index (0-5) or -1 to deselect.
     * @return True if selection was accepted.
     */
    bool setPrimaryLegSelection(int leg_index);

    /**
     * @brief Select the secondary leg for manual control (OpenSHC syropod_remote/secondary_leg_selection subscriber equivalent).
     * @param leg_index Leg index (0-5) or -1 to deselect.
     * @return True if selection was accepted.
     */
    bool setSecondaryLegSelection(int leg_index);

    /**
     * @brief Toggle primary leg state (OpenSHC syropod_remote/primary_leg_state subscriber equivalent).
     * @param state Desired leg state.
     * @return True if state change was accepted.
     */
    bool setPrimaryLegState(LegState state);

    /**
     * @brief Toggle secondary leg state (OpenSHC syropod_remote/secondary_leg_state subscriber equivalent).
     * @param state Desired leg state.
     * @return True if state change was accepted.
     */
    bool setSecondaryLegState(LegState state);

    /**
     * @brief Set primary leg tip velocity (OpenSHC syropod_remote/primary_tip_velocity subscriber equivalent).
     * @param velocity Tip velocity (mm/s).
     */
    void setPrimaryTipVelocity(const Eigen::Vector3d &velocity);

    /**
     * @brief Set secondary leg tip velocity (OpenSHC syropod_remote/secondary_tip_velocity subscriber equivalent).
     * @param velocity Tip velocity (mm/s).
     */
    void setSecondaryTipVelocity(const Eigen::Vector3d &velocity);

    /**
     * @brief Set primary leg tip pose (OpenSHC syropod_manipulation/primary_tip_pose subscriber equivalent).
     * @param pose Target tip position (mm).
     */
    void setPrimaryTipPose(const Point3D &pose);

    /**
     * @brief Set secondary leg tip pose (OpenSHC syropod_manipulation/secondary_tip_pose subscriber equivalent).
     * @param pose Target tip position (mm).
     */
    void setSecondaryTipPose(const Point3D &pose);

    /** @} */

    /** @name ROS-equivalent publication accessors (output getters).
     *
     *  These methods mirror OpenSHC ROS publisher topics, exposing internal
     *  state snapshots that external software would have obtained through
     *  topic subscription in OpenSHC.
     *  @{ */

    /**
     * @brief Get current desired velocity (OpenSHC shc/velocity publisher equivalent).
     * @return Velocity command snapshot.
     */
    VelocityCommand getDesiredVelocityCommand() const;

    /**
     * @brief Get current desired body pose (OpenSHC shc/pose publisher equivalent).
     * @return Body pose command snapshot.
     */
    BodyPoseCommand getDesiredBodyPoseCommand() const;

    /**
     * @brief Get walkspace radius data (OpenSHC shc/walkspace publisher equivalent).
     * @return Walkspace info with average, min and max radii.
     */
    WalkspaceInfo getWalkspaceInfo() const;

    /**
     * @brief Get IMU rotation pose error (OpenSHC shc/rotation_pose_error publisher equivalent).
     * @return Rotation pose error triple.
     */
    RotationPoseError getRotationPoseError() const;

    /**
     * @brief Get combined desired joint states for all legs (OpenSHC desired_joint_states publisher equivalent).
     * @param[out] positions  Joint positions (rad) per leg.
     * @param[out] velocities Joint velocities (rad/s) per leg.
     * @param[out] efforts    Joint efforts (driver units) per leg.
     * @return True if state was populated (false if system is not initialised).
     */
    bool getDesiredJointStates(JointAngles positions[NUM_LEGS],
                               JointAngles velocities[NUM_LEGS],
                               JointAngles efforts[NUM_LEGS]) const;

    /**
     * @brief Get comprehensive state for a single leg (OpenSHC shc/{leg_id}/state publisher equivalent).
     * @param leg_index Leg index (0-5).
     * @return Aggregated leg state info.
     */
    LegStateInfo getLegStateInfo(int leg_index) const;

    /**
     * @brief Get ideal odometry pose (OpenSHC odom_ideal TF equivalent).
     * @return Odometry pose (position + orientation).
     */
    Pose getOdometry() const;

    /**
     * @brief Get current walk state (OpenSHC walk state).
     * @return Walk state.
     */
    WalkState getWalkState() const;

    /**
     * @brief Get current posing mode.
     * @return Posing mode.
     */
    PosingMode getPosingMode() const;

    /**
     * @brief Get current pose reset mode.
     * @return Pose reset mode.
     */
    PoseResetMode getPoseResetMode() const;

    /**
     * @brief Get current active gait type.
     * @return Gait type.
     */
    GaitType getCurrentGaitType() const;

    /**
     * @brief Get primary selected leg index.
     * @return Leg index (0-5) or -1 if none selected.
     */
    int getPrimaryLegSelection() const;

    /**
     * @brief Get secondary selected leg index.
     * @return Leg index (0-5) or -1 if none selected.
     */
    int getSecondaryLegSelection() const;

    /** @} */

  private:
    /** Helper methods. */
    double constrainAngle(double angle, double min_angle, double max_angle);
    bool validateParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);

    /** Removed unused adaptive control helpers flagged by static analysis. */
};

#include "math_utils.h"

#endif /**< LOCOMOTION_SYSTEM_H */
