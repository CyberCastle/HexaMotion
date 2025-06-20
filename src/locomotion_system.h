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
 * - FSR_THRESHOLD: FSR contact detection threshold (ADC units)
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
        SENSOR_ERROR = 7 // General sensor communication error
    };

  private:
    // Robot parameters
    Parameters params;

    // Hardware interfaces
    IIMUInterface *imu_interface;
    IFSRInterface *fsr_interface;
    IServoInterface *servo_interface;

    // System states
    Eigen::Vector3f body_position;      // Body position [x,y,z]
    Eigen::Vector3f body_orientation;   // Body orientation [roll,pitch,yaw]
    Point3D leg_positions[NUM_LEGS];    // Leg positions
    JointAngles joint_angles[NUM_LEGS]; // Joint angles
    LegState leg_states[NUM_LEGS];      // Leg states

    // Gait control
    GaitType current_gait;
    float gait_phase;
    float step_height;
    float step_length;

    // Gait-specific parameters
    float stance_duration;             // Stance phase duration (0-1)
    float swing_duration;              // Swing phase duration (0-1)
    float cycle_frequency;             // Gait cycle frequency (Hz)
    float leg_phase_offsets[NUM_LEGS]; // Phase offset per leg

    // Control variables
    bool system_enabled;
    unsigned long last_update_time;
    float dt; // Delta time since last update (seconds)

    // Velocity control
    CartesianVelocityController *velocity_controller;

    // Error variables
    ErrorCode last_error;

    RobotModel model;
    PoseController *pose_ctrl;
    WalkController *walk_ctrl;
    AdmittanceController *admittance_ctrl;
    // FSR contact history for updateLegStates filtering
    float fsr_contact_history[NUM_LEGS][3];
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
     * @return True on successful initialization.
     */
    bool initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo);

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
    bool setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);
    /** Move one leg to a position in world coordinates. */
    bool setLegPosition(int leg_index, const Point3D &position);
    /** Command the default standing pose. */
    bool setStandingPose();
    /** Command a crouched pose for shutdown. */
    bool setCrouchPose();

    // Inverse kinematics
    /** Compute joint angles for a desired leg tip position. */
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);
    /** Compute tip position from current joint angles. */
    Point3D calculateForwardKinematics(int leg_index, const JointAngles &angles);

    // Jacobian calculation
    /**
     * @brief Calculate the geometric Jacobian for a leg.
     */
    Eigen::MatrixXf calculateJacobian(int leg_index, const JointAngles &angles);
    /**
     * @brief Calculate the analytic Jacobian for a leg.
     */
    Eigen::Matrix3f calculateAnalyticJacobian(int leg_index, const JointAngles &q);

    // Denavit-Hartenberg transforms
    /** Compute a DH transform matrix. */
    Eigen::Matrix4f calculateDHTransform(float a, float alpha, float d, float theta);
    /** Compute the transform from body to leg tip. */
    Eigen::Matrix4f calculateLegTransform(int leg_index, const JointAngles &angles);

    // Gait planner
    /** Select the active gait type. */
    bool setGaitType(GaitType gait);
    /** Plan the next gait step from desired velocities. */
    bool planGaitSequence(float velocity_x, float velocity_y, float angular_velocity);
    /** Update internal gait phase counters. */
    void updateGaitPhase();
    /** Compute foot trajectory for a leg at given phase. */
    Point3D calculateFootTrajectory(int leg_index, float phase);

    // Locomotion control
    /** Start walking forward indefinitely. */
    bool walkForward(float velocity);
    /** Start walking backward indefinitely. */
    bool walkBackward(float velocity);
    /** Rotate the robot in place indefinitely. */
    bool turnInPlace(float angular_velocity);
    /** Walk laterally to one side indefinitely. */
    bool walkSideways(float velocity, bool right_direction = true);
    /** Walk forward for a fixed duration. */
    bool walkForward(float velocity, float duration);
    /** Walk backward for a fixed duration. */
    bool walkBackward(float velocity, float duration);
    /** Rotate in place for a fixed duration. */
    bool turnInPlace(float angular_velocity, float duration);
    /** Walk sideways for a fixed duration. */
    bool walkSideways(float velocity, float duration, bool right_direction = true);
    /** Immediately stop all leg motion. */
    bool stopMovement();

    // Orientation control
    /** Maintain a desired body orientation. */
    bool maintainOrientation(const Eigen::Vector3f &target_orientation);
    /** Level the body if tilt is detected. */
    bool correctBodyTilt();
    /** Compute orientation error between desired and current pose. */
    Eigen::Vector3f calculateOrientationError();

    // Stability analysis
    /** Verify that current pose maintains stability margin. */
    bool checkStabilityMargin();
    /** Calculate center of pressure under the robot. */
    Eigen::Vector2f calculateCenterOfPressure();
    /** Compute a numeric stability index. */
    float calculateStabilityIndex();
    /** Enhanced stability calculation using absolute positioning data. */
    float calculateDynamicStabilityIndex();
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
    Eigen::Vector3f getBodyPosition() const { return body_position; }
    Eigen::Vector3f getBodyOrientation() const { return body_orientation; }
    LegState getLegState(int leg_index) const { return leg_states[leg_index]; }
    JointAngles getJointAngles(int leg_index) const { return joint_angles[leg_index]; }
    JointAngles getCurrentAngles(int leg_index) const { return joint_angles[leg_index]; }
    Point3D getLegPosition(int leg_index) const { return leg_positions[leg_index]; }
    float getStepHeight() const { return step_height; }
    float getStepLength() const;

    // Setters
    /** Replace the current parameter set. */
    bool setParameters(const Parameters &new_params);
    /** Set the control loop frequency. */
    bool setControlFrequency(float frequency);
    /** Configure step height and length. */
    bool setStepParameters(float height, float length);

    // Smooth trajectory configuration (OpenSHC-style movement)
    /** Configure smooth trajectory interpolation from current servo positions. */
    bool configureSmoothMovement(bool enable = true, float interpolation_speed = 0.1f, uint8_t max_steps = 20);

    /** Set body pose using smooth trajectory interpolation. */
    bool setBodyPoseSmooth(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);

    /** Set body pose immediately without trajectory interpolation. */
    bool setBodyPoseImmediate(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);

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
    float getCurrentServoSpeed(int leg_index, int joint_index) const;

    // Diagnostics
    /** Execute a basic hardware self-test. */
    bool performSelfTest();

  private:
    // Helper methods
    float constrainAngle(float angle, float min_angle, float max_angle);
    bool validateParameters();
    void initializeDefaultPose();
    void updateStepParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);
    float calculateLegReach() const;

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
