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

#include "admittance_controller.h"
#include "model.h"
#include "pose_controller.h"
#include "walk_controller.h"
#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

// Main locomotion system class
class LocomotionSystem {
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
    LegState leg_states[NUM_LEGS];      // Estados de las patas

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
    float dt; // Delta time

    // Kinematics matrices
    Eigen::MatrixXf dh_transforms[NUM_LEGS][DOF_PER_LEG];

    RobotModel model;
    PoseController *pose_ctrl;
    WalkController *walk_ctrl;
    AdmittanceController *admittance_ctrl;

    Point3D transformWorldToBody(const Point3D &p_world) const;
    bool setLegJointAngles(int leg_index, const JointAngles &q);
    void reprojectStandingFeet();

  public:
    // Constructor
    LocomotionSystem(const Parameters &params);

    // Destructor
    ~LocomotionSystem();

    // Initialization
    bool initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo);
    bool calibrateSystem();

    // Pose control
    bool setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);
    bool setLegPosition(int leg_index, const Point3D &position);
    bool setStandingPose();
    bool setCrouchPose();

    // Inverse kinematics
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);
    Point3D calculateForwardKinematics(int leg_index, const JointAngles &angles);

    // Jacobian calculation
    Eigen::MatrixXf calculateJacobian(int leg_index, const JointAngles &angles);
    Eigen::Matrix3f calculateAnalyticJacobian(int leg_index, const JointAngles &q);

    // Denavit-Hartenberg transforms
    Eigen::Matrix4f calculateDHTransform(float a, float alpha, float d, float theta);
    Eigen::Matrix4f calculateLegTransform(int leg_index, const JointAngles &angles);

    // Gait planner
    bool setGaitType(GaitType gait);
    bool planGaitSequence(float velocity_x, float velocity_y, float angular_velocity);
    void updateGaitPhase();
    Point3D calculateFootTrajectory(int leg_index, float phase);

    // Locomotion control
    bool walkForward(float velocity);
    bool walkBackward(float velocity);
    bool turnInPlace(float angular_velocity);
    bool walkSideways(float velocity, bool right_direction = true);
    bool walkForward(float velocity, float duration);
    bool walkBackward(float velocity, float duration);
    bool turnInPlace(float angular_velocity, float duration);
    bool walkSideways(float velocity, float duration, bool right_direction = true);
    bool stopMovement();

    // Orientation control
    bool maintainOrientation(const Eigen::Vector3f &target_orientation);
    bool correctBodyTilt();
    Eigen::Vector3f calculateOrientationError();

    // Stability analysis
    bool checkStabilityMargin();
    Eigen::Vector2f calculateCenterOfPressure();
    float calculateStabilityIndex();
    bool isStaticallyStable();

    // Error control
    enum ErrorCode {
        NO_ERROR = 0,
        IMU_ERROR = 1,
        FSR_ERROR = 2,
        SERVO_ERROR = 3,
        KINEMATICS_ERROR = 4,
        STABILITY_ERROR = 5,
        PARAMETER_ERROR = 6
    };

    ErrorCode getLastError() const { return last_error; }
    String getErrorMessage(ErrorCode error);
    bool handleError(ErrorCode error);

    // System update
    bool update();

    // Getters
    const Parameters &getParameters() const { return params; }
    Eigen::Vector3f getBodyPosition() const { return body_position; }
    Eigen::Vector3f getBodyOrientation() const { return body_orientation; }
    LegState getLegState(int leg_index) const { return leg_states[leg_index]; }
    JointAngles getJointAngles(int leg_index) const { return joint_angles[leg_index]; }
    Point3D getLegPosition(int leg_index) const { return leg_positions[leg_index]; }
    float getStepHeight() const { return step_height; }
    float getStepLength() const;

    // Setters
    bool setParameters(const Parameters &new_params);
    bool setControlFrequency(float frequency);
    bool setStepParameters(float height, float length);

    // Diagnostics
    bool performSelfTest();

  private:
    // Error variables
    ErrorCode last_error;

    // Helper methods
    float constrainAngle(float angle, float min_angle, float max_angle);
    bool validateParameters();
    void initializeDefaultPose();
    void updateLegStates();
    void updateStepParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);
    float calculateLegReach(int leg_index);

    // Adaptive control
    void adaptGaitToTerrain();
    void adjustStepParameters();
    void compensateForSlope();
};

#include "math_utils.h"

#endif // LOCOMOTION_SYSTEM_H
