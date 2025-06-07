/**
 * @file hexapod_locomotion_system.h
 * @brief Sistema de Control de Locomoción para Robot Hexápodo basado en OpenSHC
 * @author BlightHunter Team
 * @version 1.0
 * @date 2024
 *
 * PARÁMETROS DE ENTRADA REQUERIDOS:
 * ================================
 *
 * DIMENSIONES FÍSICAS:
 * - HEXAGON_RADIUS: Radio del hexágono del cuerpo (mm) [Típico: 150-200mm]
 * - COXA_LENGTH: Longitud del segmento coxa (mm) [Típico: 40-60mm]
 * - FEMUR_LENGTH: Longitud del segmento fémur (mm) [Típico: 80-120mm]
 * - TIBIA_LENGTH: Longitud del segmento tibia (mm) [Típico: 100-150mm]
 *
 * ÁNGULOS DE CONFIGURACIÓN:
 * - LEG_ANGLE_OFFSET: Ángulo entre patas (60° para hexápodo)
 * - COXA_ANGLE_LIMITS: Límites de ángulo coxa [min_angle, max_angle] (grados)
 * - FEMUR_ANGLE_LIMITS: Límites de ángulo fémur [min_angle, max_angle] (grados)
 * - TIBIA_ANGLE_LIMITS: Límites de ángulo tibia [min_angle, max_angle] (grados)
 *
 * CARACTERÍSTICAS DEL ROBOT:
 * - ROBOT_HEIGHT: Altura nominal del robot (mm) [Típico: 80-150mm]
 * - ROBOT_WEIGHT: Peso del robot (kg) [Para cálculos de estabilidad]
 * - CENTER_OF_MASS: Coordenadas del centro de masa [x, y, z] (mm)
 *
 * PARÁMETROS DENAVIT-HARTENBERG:
 * - DH_PARAMETERS: Matriz 6x4 con parámetros [a, alpha, d, theta] para cada articulación
 *
 * CONFIGURACIÓN DE SENSORES:
 * - IMU_CALIBRATION_OFFSET: Offset de calibración del IMU [roll, pitch, yaw] (grados)
 * - FSR_THRESHOLD: Umbral de detección de contacto FSR (unidades ADC)
 * - FSR_MAX_PRESSURE: Presión máxima detectable FSR (N o kg)
 *
 * PARÁMETROS DE CONTROL:
 * - MAX_VELOCITY: Velocidad máxima de locomoción (mm/s)
 * - MAX_ANGULAR_VELOCITY: Velocidad angular máxima (grados/s)
 * - STABILITY_MARGIN: Margen de estabilidad mínimo (mm)
 * - CONTROL_FREQUENCY: Frecuencia de control (Hz) [Típico: 50-100Hz]
 */

#ifndef HEXAPOD_LOCOMOTION_SYSTEM_H
#define HEXAPOD_LOCOMOTION_SYSTEM_H

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <math.h>

// Configuración del sistema
#define NUM_LEGS 6
#define DOF_PER_LEG 3
#define TOTAL_DOF (NUM_LEGS * DOF_PER_LEG)

// Parámetros físicos configurables
struct HexapodParameters {
    // Dimensiones físicas
    float hexagon_radius; // Radio del hexágono (mm)
    float coxa_length;    // Longitud coxa (mm)
    float femur_length;   // Longitud fémur (mm)
    float tibia_length;   // Longitud tibia (mm)

    // Características del robot
    float robot_height;             // Altura nominal (mm)
    float robot_weight;             // Peso (kg)
    Eigen::Vector3f center_of_mass; // Centro de masa [x,y,z] (mm)

    // Límites angulares (grados)
    float coxa_angle_limits[2];  // [min, max]
    float femur_angle_limits[2]; // [min, max]
    float tibia_angle_limits[2]; // [min, max]

    // Parámetros DH (a, alpha, d, theta_offset)
    float dh_parameters[NUM_LEGS][DOF_PER_LEG][4];

    // Configuración de sensores
    Eigen::Vector3f imu_calibration_offset; // [roll, pitch, yaw] (grados)
    float fsr_threshold;                    // Umbral contacto FSR
    float fsr_max_pressure;                 // Presión máxima FSR

    // Parámetros de control
    float max_velocity;         // Velocidad máxima (mm/s)
    float max_angular_velocity; // Velocidad angular máxima (grados/s)
    float stability_margin;     // Margen de estabilidad (mm)
    float control_frequency;    // Frecuencia de control (Hz)

    // Parámetros de cinemática inversa
    struct IKConfig {
        uint8_t max_iterations = 30;   // OpenSHC default
        float pos_threshold_mm = 0.5f; //   ”      ”
        bool use_damping = true;
        float damping_lambda = 30.0f; // λ ≈ 30mm  (see paper)
        bool clamp_joints = true;     // ik_clamp_joints
    } ik;

    // Parametros de compensación del cuerpo
    struct BodyCompConfig {
        bool enable = true;         // “IMU body compensation”
        float kp = 0.6f;            // same order as OpenSHC demos
        float lp_alpha = 0.10f;     // 1-pole low-pass
        float max_tilt_deg = 12.0f; // disable beyond this (OpenSHC default)
    } body_comp;
};

// Enumeraciones para tipos de marcha
enum GaitType {
    TRIPOD_GAIT,
    WAVE_GAIT,
    RIPPLE_GAIT,
    METACHRONAL_GAIT,
    ADAPTIVE_GAIT
};

// Estados de las patas
enum LegState {
    STANCE_PHASE,   // Pata en contacto con suelo
    SWING_PHASE,    // Pata en movimiento
    LIFT_PHASE,     // Levantando pata
    TOUCHDOWN_PHASE // Bajando pata
};

// Estructura para punto 3D
struct Point3D {
    float x, y, z;
    Point3D(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}
};

// Estructura para ángulos de articulación
struct JointAngles {
    float coxa, femur, tibia;
    JointAngles(float c = 0, float f = 0, float t = 0) : coxa(c), femur(f), tibia(t) {}
};

// Estructura para datos del IMU
struct IMUData {
    float roll, pitch, yaw;          // Orientación (grados)
    float accel_x, accel_y, accel_z; // Aceleración (m/s²)
    float gyro_x, gyro_y, gyro_z;    // Velocidad angular (grados/s)
    bool is_valid;
};

// Estructura para datos FSR
struct FSRData {
    float pressure;     // Presión detectada
    bool in_contact;    // Estado de contacto
    float contact_time; // Tiempo en contacto (ms)
};

// Interfaces para hardware (a implementar por el usuario)
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

// Clase principal del sistema de locomoción
class HexapodLocomotionSystem {
  private:
    // Parámetros del robot
    HexapodParameters params;

    // Interfaces de hardware
    IIMUInterface *imu_interface;
    IFSRInterface *fsr_interface;
    IServoInterface *servo_interface;

    // Estados del sistema
    Eigen::Vector3f body_position;      // Posición del cuerpo [x,y,z]
    Eigen::Vector3f body_orientation;   // Orientación del cuerpo [roll,pitch,yaw]
    Point3D leg_positions[NUM_LEGS];    // Posiciones de las patas
    JointAngles joint_angles[NUM_LEGS]; // Ángulos de articulaciones
    LegState leg_states[NUM_LEGS];      // Estados de las patas

    // Control de marcha
    GaitType current_gait;
    float gait_phase;
    float step_height;
    float step_length;

    // Parámetros específicos de gait
    float stance_duration;             // Duración de fase de apoyo (0-1)
    float swing_duration;              // Duración de fase de vuelo (0-1)
    float cycle_frequency;             // Frecuencia del ciclo de gait (Hz)
    float leg_phase_offsets[NUM_LEGS]; // Offset de fase por pata

    // Variables de control
    bool system_enabled;
    unsigned long last_update_time;
    float dt; // Delta time

    // Matrices para cinemática
    Eigen::MatrixXf dh_transforms[NUM_LEGS][DOF_PER_LEG];

    Point3D transformWorldToBody(const Point3D &p_world) const;
    bool setLegJointAngles(int leg_index, const JointAngles &q);
    void reprojectStandingFeet();

  public:
    // Constructor
    HexapodLocomotionSystem(const HexapodParameters &params);

    // Destructor
    ~HexapodLocomotionSystem();

    // Inicialización
    bool initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo);
    bool calibrateSystem();

    // Control de pose
    bool setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation);
    bool setLegPosition(int leg_index, const Point3D &position);
    bool setStandingPose();
    bool setCrouchPose();

    // Cinemática inversa
    JointAngles calculateInverseKinematics(int leg_index, const Point3D &target_position);
    Point3D calculateForwardKinematics(int leg_index, const JointAngles &angles);

    // Cálculo de jacobianos
    Eigen::MatrixXf calculateJacobian(int leg_index, const JointAngles &angles);
    Eigen::Matrix3f calculateAnalyticJacobian(int leg_index, const JointAngles &q);

    // Transformaciones Denavit-Hartenberg
    Eigen::Matrix4f calculateDHTransform(float a, float alpha, float d, float theta);
    Eigen::Matrix4f calculateLegTransform(int leg_index, const JointAngles &angles);

    // Planificador de marchas
    bool setGaitType(GaitType gait);
    bool planGaitSequence(float velocity_x, float velocity_y, float angular_velocity);
    void updateGaitPhase();
    Point3D calculateFootTrajectory(int leg_index, float phase);

    // Control de locomoción
    bool walkForward(float velocity);
    bool walkBackward(float velocity);
    bool turnInPlace(float angular_velocity);
    bool walkSideways(float velocity, bool right_direction = true);
    bool walkForward(float velocity, float duration);
    bool walkBackward(float velocity, float duration);
    bool turnInPlace(float angular_velocity, float duration);
    bool walkSideways(float velocity, float duration, bool right_direction = true);
    bool stopMovement();

    // Control de orientación
    bool maintainOrientation(const Eigen::Vector3f &target_orientation);
    bool correctBodyTilt();
    Eigen::Vector3f calculateOrientationError();

    // Análisis de estabilidad
    bool checkStabilityMargin();
    Eigen::Vector2f calculateCenterOfPressure();
    float calculateStabilityIndex();
    bool isStaticallyStable();

    // Control de errores
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

    // Actualización del sistema
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

    // Diagnóstico
    bool performSelfTest();
    void printSystemStatus();
    void printLegStatus(int leg_index);

  private:
    // Variables de error
    ErrorCode last_error;

    // Métodos auxiliares
    float constrainAngle(float angle, float min_angle, float max_angle);
    bool validateParameters();
    void initializeDefaultPose();
    void updateLegStates();
    void updateStepParameters();
    bool checkJointLimits(int leg_index, const JointAngles &angles);
    float calculateLegReach(int leg_index);

    // Control adaptativo
    void adaptGaitToTerrain();
    void adjustStepParameters();
    void compensateForSlope();
};

// Funciones de utilidad
namespace HexapodUtils {
float degreesToRadians(float degrees);
float radiansToDegrees(float radians);
float normalizeAngle(float angle);
Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation);
float distance3D(const Point3D &p1, const Point3D &p2);
bool isPointReachable(const Point3D &point, float max_reach);

// Funciones matemáticas de matrices de rotación
Eigen::Matrix3f rotationMatrixX(float angle);
Eigen::Matrix3f rotationMatrixY(float angle);
Eigen::Matrix3f rotationMatrixZ(float angle);

// Cálculos matemáticos
Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion);
Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler);
Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2);
Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q);

} // namespace HexapodUtils

#endif // HEXAPOD_LOCOMOTION_SYSTEM_H