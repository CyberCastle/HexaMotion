/**
 * @file locomotion_system.h
 * @brief Locomotion Control System inspired by OpenSHC
 * @author BlightHunter Team
 * @version 1.0
 * @date 2024
 *
 * REQUIRED INPUT PARAMETERS:
 * ==========================
 *
 * PHYSICAL DIMENSIONS:
 * - HEXAGON_RADIUS: Body hexagon radius (mm) [Typical: 150-200mm]
 * - COXA_LENGTH: Coxa segment length (mm) [Typical: 40-60mm]
 * - FEMUR_LENGTH: Femur segment length (mm) [Typical: 80-120mm]
 * - TIBIA_LENGTH: Tibia segment length (mm) [Typical: 100-150mm]
 *
 * CONFIGURATION ANGLES:
 * - LEG_ANGLE_OFFSET: Angle between legs (60° for hexapod)
 * - COXA_ANGLE_LIMITS: Coxa angle limits [min_angle, max_angle] (degrees)
 * - FEMUR_ANGLE_LIMITS: Femur angle limits [min_angle, max_angle] (degrees)
 * - TIBIA_ANGLE_LIMITS: Tibia angle limits [min_angle, max_angle] (degrees)
 *
 * ROBOT CHARACTERISTICS:
 * - ROBOT_HEIGHT: Nominal robot height (mm) [Typical: 80-150mm]
 * - ROBOT_WEIGHT: Robot weight (kg) [Used for stability calculations]
 * - CENTER_OF_MASS: Center of mass coordinates [x, y, z] (mm)
 *
 * DENAVIT-HARTENBERG PARAMETERS:
 * - DH_PARAMETERS: 6x4 matrix with [a, alpha, d, theta] for each joint
 *
 * SENSOR SETTINGS:
 * - IMU_CALIBRATION_OFFSET: IMU calibration offset [roll, pitch, yaw] (degrees)
 * - FSR_TOUCHDOWN_THRESHOLD: FSR touchdown detection threshold (ADC units)
 * - FSR_LIFTOFF_THRESHOLD: FSR liftoff detection threshold (ADC units)
 * - FSR_MAX_PRESSURE: Maximum detectable FSR pressure (N or kg)
 *
 * CONTROL PARAMETERS:
 * - MAX_VELOCITY: Maximum locomotion speed (mm/s)
 * - MAX_ANGULAR_VELOCITY: Maximum angular velocity (degrees/s)
 * - STABILITY_MARGIN: Minimum stability margin (mm)
 * - CONTROL_FREQUENCY: Control frequency (Hz) [Typical: 50-100Hz]
 */

#ifndef LOCOMOTION_SYSTEM_H
#define LOCOMOTION_SYSTEM_H

#include "HexaModel.h"
#include "admittance_controller.h"
#include "cartesian_velocity_controller.h"
#include "pose_controller.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

// Main locomotion system class
class LocomotionSystem {
  public:
    // Error control
    enum ErrorCode {
        NO_ERROR = 0,
        IMU_ERROR = 1,
        FSR_ERROR = 2,
        SERVO_ERROR = 3,
        KINEMATICS_ERROR = 4,
        STABILITY_ERROR = 5,
        PARAMETER_ERROR = 6,
        SENSOR_ERROR = 7,       // General sensor communication error
        SERVO_BLOCKED_ERROR = 8 // Servo blocked by status flags
    };

  private:
    // Robot parameters
    Parameters params;

    // Hardware interfaces
    IIMUInterface *imu_interface;
    IFSRInterface *fsr_interface;
    IServoInterface *servo_interface;

    // System states
    Eigen::Vector3d body_position;      // Body position [x,y,z]
    Eigen::Vector3d body_orientation;   // Body orientation [roll,pitch,yaw]
    Point3D leg_positions[NUM_LEGS];    // Leg positions
    JointAngles joint_angles[NUM_LEGS]; // Joint angles
    LegState leg_states[NUM_LEGS];      // Leg states

    // Gait control
    GaitType current_gait;
    double gait_phase;
    double step_height;
    double step_length;

    // Gait-specific parameters
    double stance_duration;             // Stance phase duration (0-1)
    double swing_duration;              // Swing phase duration (0-1)
    double cycle_frequency;             // Gait cycle frequency (Hz)
    double leg_phase_offsets[NUM_LEGS]; // Phase offset per leg

    // Control variables
    bool system_enabled;
    unsigned long last_update_time;
    double dt; // Delta time since last update (seconds)

    // Velocity control
    CartesianVelocityController *velocity_controller;

    // Error variables
    ErrorCode last_error;

    RobotModel model;
    PoseController *pose_ctrl;
    WalkController *walk_ctrl;
    AdmittanceController *admittance_ctrl;
    // FSR contact history for updateLegStates filtering
    double fsr_contact_history[NUM_LEGS][3];
    // Circular buffer index for FSR history
    int fsr_history_index;
    // Last log time for sensor update profiling
    unsigned long last_sensor_log_time;

    Point3D transformWorldToBody(const Point3D &p_world) const;
    bool setLegJointAngles(int leg_index, const JointAngles &q);

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
    bool initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo, const PoseConfiguration &pose_config);

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

    // Pose control
    /** Set the full body pose. */
    bool setBodyPose(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);
    /** Move one leg to a position in world coordinates. */
    bool setLegPosition(int leg_index, const Point3D &position);
    /** Command the default standing pose. */
    bool setStandingPose();

    // Inverse kinematics
    /** Compute joint angles for a desired leg tip position. */
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);
    /** Compute tip position from current joint angles. */
    Point3D calculateForwardKinematics(int leg_index, const JointAngles &angles);

    /** Check if target is within workspace. */
    bool isTargetReachable(int leg_index, const Point3D &target);
    /** Project target to workspace boundary if outside. */
    Point3D constrainToWorkspace(int leg_index, const Point3D &target);
    /** Get joint limit proximity (1.0 = far from limits, 0.0 = at limits). */
    double getJointLimitProximity(int leg_index, const JointAngles &angles);

    // Denavit-Hartenberg transforms
    /** Compute a DH transform matrix. */
    Eigen::Matrix4d calculateDHTransform(double a, double alpha, double d, double theta);
    /** Compute the transform from body to leg tip. */
    Eigen::Matrix4d calculateLegTransform(int leg_index, const JointAngles &angles);

    // Gait planner
    /** Select the active gait type. */
    bool setGaitType(GaitType gait);
    /** Plan the next gait step from desired velocities. */
    bool planGaitSequence(double velocity_x, double velocity_y, double angular_velocity);
    /** Update internal gait phase counters. */
    void updateGaitPhase();
    /** Compute foot trajectory for a leg at given phase. */
    Point3D calculateFootTrajectory(int leg_index, double phase);

    // Locomotion control
    /** Start walking forward indefinitely. */
    bool walkForward(double velocity);
    /** Start walking backward indefinitely. */
    bool walkBackward(double velocity);
    /** Rotate the robot in place indefinitely. */
    bool turnInPlace(double angular_velocity);
    /** Walk laterally to one side indefinitely. */
    bool walkSideways(double velocity, bool right_direction = true);
    /** Walk forward for a fixed duration. */
    bool walkForward(double velocity, double duration);
    /** Walk backward for a fixed duration. */
    bool walkBackward(double velocity, double duration);
    /** Rotate in place for a fixed duration. */
    bool turnInPlace(double angular_velocity, double duration);
    /** Walk sideways for a fixed duration. */
    bool walkSideways(double velocity, double duration, bool right_direction = true);
    /** Immediately stop all leg motion. */
    bool stopMovement();

    // Orientation control
    /** Maintain a desired body orientation. */
    bool maintainOrientation(const Eigen::Vector3d &target_orientation);
    /** Level the body if tilt is detected. */
    bool correctBodyTilt();
    /** Compute orientation error between desired and current pose. */
    Eigen::Vector3d calculateOrientationError();

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

    ErrorCode getLastError() const { return last_error; }
    String getErrorMessage(ErrorCode error);
    bool handleError(ErrorCode error);

    // Leg state management
    /** Update leg contact states based on FSR sensor readings. */
    void updateLegStates();
    /** Reproject standing feet positions during body orientation changes. */
    void reprojectStandingFeet();

    // System update
    /** Update all controllers and state machines. */
    bool update();

    /** Update FSR and IMU sensors in parallel for optimal performance. */
    bool updateSensorsParallel();

    // Getters
    const Parameters &getParameters() const { return params; }
    RobotModel &getRobotModel() { return model; }
    const RobotModel &getRobotModel() const { return model; }
    IServoInterface *getServoInterface() { return servo_interface; }
    Eigen::Vector3d getBodyPosition() const { return body_position; }
    Eigen::Vector3d getBodyOrientation() const { return body_orientation; }
    LegState getLegState(int leg_index) const { return leg_states[leg_index]; }
    JointAngles getJointAngles(int leg_index) const { return joint_angles[leg_index]; }
    JointAngles getCurrentAngles(int leg_index) const { return joint_angles[leg_index]; }
    Point3D getLegPosition(int leg_index) const { return leg_positions[leg_index]; }
    double getStepHeight() const { return step_height; }
    double getStepLength() const;

    // Setters
    /** Replace the current parameter set. */
    bool setParameters(const Parameters &new_params);
    /** Set the control loop frequency. */
    bool setControlFrequency(double frequency);
    /** Configure step height and length. */
    bool setStepParameters(double height, double length);

    // Smooth trajectory configuration (OpenSHC-style movement)
    /** Configure smooth trajectory interpolation from current servo positions. */
    bool configureSmoothMovement(bool enable = true, double interpolation_speed = 0.1f, uint8_t max_steps = 20);

    /** Set body pose using smooth trajectory interpolation. */
    bool setBodyPoseSmooth(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /** Set body pose immediately without trajectory interpolation. */
    bool setBodyPoseImmediate(const Eigen::Vector3d &position, const Eigen::Vector3d &orientation);

    /** Check if a smooth movement trajectory is currently in progress. */
    bool isSmoothMovementInProgress() const;

    /** Reset any active smooth movement trajectory. */
    void resetSmoothMovement();

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

    // Diagnostics
    /** Execute a basic hardware self-test. */
    bool performSelfTest();

  private:
    // Helper methods
    double constrainAngle(double angle, double min_angle, double max_angle);
    bool validateParameters();
    void updateStepParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);
    double calculateLegReach() const;

    // Adaptive control
    void adaptGaitToTerrain();
    void adjustStepParameters();
    void compensateForSlope();

    // Advanced gait methods
    void updateMetachronalPattern();
    void updateAdaptivePattern();
    bool shouldAdaptGaitPattern();
    void calculateAdaptivePhaseOffsets();
};

#include "math_utils.h"

#endif // LOCOMOTION_SYSTEM_H
