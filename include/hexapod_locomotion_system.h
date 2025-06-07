/**
 * @file hexapod_locomotion_system.h
 * @brief Hexapod Locomotion Control System inspired by OpenSHC
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

#ifndef HEXAPOD_LOCOMOTION_SYSTEM_H
#define HEXAPOD_LOCOMOTION_SYSTEM_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

// System configuration
#define NUM_LEGS 6
#define DOF_PER_LEG 3
#define TOTAL_DOF (NUM_LEGS * DOF_PER_LEG)

// Configurable physical parameters
struct HexapodParameters {
    // Physical dimensions
    float hexagon_radius; // Hexagon radius (mm)
    float coxa_length;    // Coxa length (mm)
    float femur_length;   // Femur length (mm)
    float tibia_length;   // Tibia length (mm)

    // Robot characteristics
    float robot_height;             // Nominal height (mm)
    float robot_weight;             // Weight (kg)
    Eigen::Vector3f center_of_mass; // Center of mass [x,y,z] (mm)

    // Angle limits (degrees)
    float coxa_angle_limits[2];  // [min, max]
    float femur_angle_limits[2]; // [min, max]
    float tibia_angle_limits[2]; // [min, max]

    // DH parameters (a, alpha, d, theta_offset)
    float dh_parameters[NUM_LEGS][DOF_PER_LEG][4];

    // Sensor configuration
    Eigen::Vector3f imu_calibration_offset; // [roll, pitch, yaw] (grados)
    float fsr_threshold;                    // FSR contact threshold
    float fsr_max_pressure;                 // Maximum FSR pressure

    // Control parameters
    float max_velocity;         // Maximum speed (mm/s)
    float max_angular_velocity; // Maximum angular speed (degrees/s)
    float stability_margin;     // Stability margin (mm)
    float control_frequency;    // Control frequency (Hz)

    // Inverse kinematics parameters
    struct IKConfig {
        uint8_t max_iterations = 30;   // OpenSHC default
        float pos_threshold_mm = 0.5f; //   ”      ”
        bool use_damping = true;
        float damping_lambda = 30.0f; // λ ≈ 30mm  (see paper)
        bool clamp_joints = true;     // ik_clamp_joints
    } ik;

    // Body compensation parameters
    struct BodyCompConfig {
        bool enable = true;         // “IMU body compensation”
        float kp = 0.6f;            // same order as OpenSHC demos
        float lp_alpha = 0.10f;     // 1-pole low-pass
        float max_tilt_deg = 12.0f; // disable beyond this (OpenSHC default)
    } body_comp;
};

// Gait type enumerations
enum GaitType {
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

// Leg states
enum LegState {
    STANCE_PHASE,   // Leg on ground
    SWING_PHASE,    // Leg moving
    LIFT_PHASE,     // Lifting leg
    TOUCHDOWN_PHASE // Lowering leg
};

// 3D point structure
struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

// Joint angles structure
struct JointAngles {
    float coxa, femur, tibia;
    JointAngles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

// IMU data structure
struct IMUData {
    float roll, pitch, yaw;          // Orientation (degrees)
    float accel_x, accel_y, accel_z; // Acceleration (m/s²)
    float gyro_x, gyro_y, gyro_z;    // Angular velocity (degrees/s)
    bool is_valid;
};

// FSR data structure
struct FSRData {
    float pressure;     // Measured pressure
    bool in_contact;    // Contact state
    float contact_time; // Contact time (ms)
};

// Hardware interfaces (to be implemented by the user)
class IIMUInterface {
  public:
    virtual ~IIMUInterface() = default;
    virtual bool initialize() = 0;
    virtual IMUData readIMU() = 0;
    virtual bool calibrate() = 0;
    virtual bool isConnected() = 0;
};

class IFSRInterface {
  public:
    virtual ~IFSRInterface() = default;
    virtual bool initialize() = 0;
    virtual FSRData readFSR(int leg_index) = 0;
    virtual bool calibrateFSR(int leg_index) = 0;
    virtual float getRawReading(int leg_index) = 0;
};

class IServoInterface {
  public:
    virtual ~IServoInterface() = default;
    virtual bool initialize() = 0;
    virtual bool setJointAngle(int leg_index, int joint_index, float angle) = 0;
    virtual float getJointAngle(int leg_index, int joint_index) = 0;
    virtual bool setJointSpeed(int leg_index, int joint_index, float speed) = 0;
    virtual bool isJointMoving(int leg_index, int joint_index) = 0;
    virtual bool enableTorque(int leg_index, int joint_index, bool enable) = 0;
};

// Main locomotion system class
class HexapodLocomotionSystem {
  private:
    // Robot parameters
    HexapodParameters params;

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

    Point3D transformWorldToBody(const Point3D &p_world) const;
    bool setLegJointAngles(int leg_index, const JointAngles &q);
    void reprojectStandingFeet();

  public:
    // Constructor
    HexapodLocomotionSystem(const HexapodParameters &params);

    // Destructor
    ~HexapodLocomotionSystem();

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
    const HexapodParameters &getParameters() const { return params; }
    Eigen::Vector3f getBodyPosition() const { return body_position; }
    Eigen::Vector3f getBodyOrientation() const { return body_orientation; }
    LegState getLegState(int leg_index) const { return leg_states[leg_index]; }
    JointAngles getJointAngles(int leg_index) const { return joint_angles[leg_index]; }
    Point3D getLegPosition(int leg_index) const { return leg_positions[leg_index]; }

    // Setters
    bool setParameters(const HexapodParameters &new_params);
    bool setControlFrequency(float frequency);
    bool setStepParameters(float height, float length);

    // Diagnostics
    bool performSelfTest();
    void printSystemStatus();
    void printLegStatus(int leg_index);

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

// Utility functions
namespace HexapodUtils {
float degreesToRadians(float degrees);
float radiansToDegrees(float radians);
float normalizeAngle(float angle);
Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation);
float distance3D(const Point3D &p1, const Point3D &p2);
bool isPointReachable(const Point3D &point, float max_reach);

// Rotation matrix functions
Eigen::Matrix3f rotationMatrixX(float angle);
Eigen::Matrix3f rotationMatrixY(float angle);
Eigen::Matrix3f rotationMatrixZ(float angle);

// Mathematical calculations
Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion);
Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler);
Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2);
Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q);

} // namespace HexapodUtils

#endif // HEXAPOD_LOCOMOTION_SYSTEM_H
