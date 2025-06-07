/**
 * @file hexapod_locomotion_system.cpp
 * @brief Implementation of the Hexapod Locomotion Control System
 * @author BlightHunter Team
 * @version 1.0
 * @date 2024
 *
 * Implements control based on:
 * - Inverse kinematics using Denavit-Hartenberg parameters
 * - Jacobians for velocity control
 * - Gait planner with multiple gaits
 * - Orientation and stability control
 * - Principles of OpenSHC (Open Source Humanoid Control)
 */

#include "hexapod_locomotion_system.h"
#include <vector>

// Funciones de utilidad
namespace HexapodUtils {
float degreesToRadians(float degrees) {
    return degrees * M_PI / 180.0f;
}

float radiansToDegrees(float radians) {
    return radians * 180.0f / M_PI;
}

float normalizeAngle(float angle) {
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

Point3D rotatePoint(const Point3D &point, const Eigen::Vector3f &rotation) {
    // Use rotation matrices from this namespace
    Eigen::Matrix3f Rx = rotationMatrixX(rotation[0]);
    Eigen::Matrix3f Ry = rotationMatrixY(rotation[1]);
    Eigen::Matrix3f Rz = rotationMatrixZ(rotation[2]);

    Eigen::Matrix3f R = Rz * Ry * Rx;
    Eigen::Vector3f p(point.x, point.y, point.z);
    Eigen::Vector3f rotated = R * p;

    return Point3D(rotated[0], rotated[1], rotated[2]);
}

float distance3D(const Point3D &p1, const Point3D &p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

bool isPointReachable(const Point3D &point, float max_reach) {
    float distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    return distance <= max_reach;
}

// Helper math function implementations
Eigen::Vector4f quaternionMultiply(const Eigen::Vector4f &q1, const Eigen::Vector4f &q2) {
    Eigen::Vector4f result;
    result[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]; // w
    result[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]; // x
    result[2] = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]; // y
    result[3] = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]; // z
    return result;
}

Eigen::Vector4f quaternionInverse(const Eigen::Vector4f &q) {
    float norm_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (norm_sq > 0) {
        return Eigen::Vector4f(q[0] / norm_sq, -q[1] / norm_sq, -q[2] / norm_sq, -q[3] / norm_sq);
    }
    return q; // Return original if degenerate
}

Eigen::Vector3f quaternionToEuler(const Eigen::Vector4f &quaternion) {
    Eigen::Vector3f euler;
    float w = quaternion[0], x = quaternion[1], y = quaternion[2], z = quaternion[3];

    // Roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    euler[0] = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        euler[1] = copysign(M_PI / 2, sinp);
    else
        euler[1] = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    euler[2] = atan2(siny_cosp, cosy_cosp);

    // Convertir a grados
    euler[0] = HexapodUtils::radiansToDegrees(euler[0]);
    euler[1] = HexapodUtils::radiansToDegrees(euler[1]);
    euler[2] = HexapodUtils::radiansToDegrees(euler[2]);

    return euler;
}

Eigen::Vector4f eulerToQuaternion(const Eigen::Vector3f &euler) {
    float roll = HexapodUtils::degreesToRadians(euler[0]);
    float pitch = HexapodUtils::degreesToRadians(euler[1]);
    float yaw = HexapodUtils::degreesToRadians(euler[2]);

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    Eigen::Vector4f q;
    q[0] = cr * cp * cy + sr * sp * sy; // w
    q[1] = sr * cp * cy - cr * sp * sy; // x
    q[2] = cr * sp * cy + sr * cp * sy; // y
    q[3] = cr * cp * sy - sr * sp * cy; // z

    return q;
}

Eigen::Matrix3f rotationMatrixX(float angle) {
    float rad = HexapodUtils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << 1, 0, 0,
        0, cos(rad), -sin(rad),
        0, sin(rad), cos(rad);
    return R;
}

Eigen::Matrix3f rotationMatrixY(float angle) {
    float rad = HexapodUtils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << cos(rad), 0, sin(rad),
        0, 1, 0,
        -sin(rad), 0, cos(rad);
    return R;
}

Eigen::Matrix3f rotationMatrixZ(float angle) {
    float rad = HexapodUtils::degreesToRadians(angle);
    Eigen::Matrix3f R;
    R << cos(rad), -sin(rad), 0,
        sin(rad), cos(rad), 0,
        0, 0, 1;
    return R;
}

} // namespace HexapodUtils

// Constructor
HexapodLocomotionSystem::HexapodLocomotionSystem(const HexapodParameters &params)
    : params(params), imu_interface(nullptr), fsr_interface(nullptr), servo_interface(nullptr),
      current_gait(TRIPOD_GAIT), gait_phase(0.0f), step_height(30.0f), step_length(50.0f),
      stance_duration(0.5f), swing_duration(0.5f), cycle_frequency(2.0f),
      system_enabled(false), last_update_time(0), dt(0.02f), last_error(NO_ERROR) {

    // Initialize body position
    body_position = Eigen::Vector3f(0.0f, 0.0f, params.robot_height);
    body_orientation = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    // Inicializar estados de las patas
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states[i] = STANCE_PHASE;
        leg_phase_offsets[i] = (i % 2) * 0.5f; // Inicializar con TRIPOD por defecto
    }

    initializeDefaultPose();
}

// Destructor
HexapodLocomotionSystem::~HexapodLocomotionSystem() {
    system_enabled = false;
}

// System initialization
bool HexapodLocomotionSystem::initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo) {
    if (!imu || !fsr || !servo) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    imu_interface = imu;
    fsr_interface = fsr;
    servo_interface = servo;

    // Inicializar interfaces
    if (!imu_interface->initialize()) {
        last_error = IMU_ERROR;
        return false;
    }

    if (!fsr_interface->initialize()) {
        last_error = FSR_ERROR;
        return false;
    }

    if (!servo_interface->initialize()) {
        last_error = SERVO_ERROR;
        return false;
    }

    // Validar parameters
    if (!validateParameters()) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    system_enabled = true;
    last_update_time = millis();

    return true;
}

// System calibration
bool HexapodLocomotionSystem::calibrateSystem() {
    if (!system_enabled)
        return false;

    // Calibrar IMU
    if (!imu_interface->calibrate()) {
        last_error = IMU_ERROR;
        return false;
    }

    // Calibrar FSRs
    for (int i = 0; i < NUM_LEGS; i++) {
        if (!fsr_interface->calibrateFSR(i)) {
            last_error = FSR_ERROR;
            return false;
        }
    }

    // Establecer pose inicial
    setStandingPose();

    return true;
}

// Inverse kinematics using an optimized geometric method
JointAngles HexapodLocomotionSystem::calculateInverseKinematics(int leg,
                                                                const Point3D &p_target) {
    JointAngles q = joint_angles[leg]; // semilla “cercana”
    const uint8_t N = params.ik.max_iterations;
    const float eps = params.ik.pos_threshold_mm;

    for (uint8_t it = 0; it < N; ++it) {
        Point3D p_now = calculateForwardKinematics(leg, q);
        Eigen::Vector3f e(p_target.x - p_now.x,
                          p_target.y - p_now.y,
                          p_target.z - p_now.z);

        if (e.norm() < eps)
            return q; // ¡converge!

        Eigen::Matrix3f J = calculateAnalyticJacobian(leg, q);

        //  Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e      (Levenberg-Marquardt) 
        const float lambda = params.ik.use_damping ? params.ik.damping_lambda : 0.0f;
        Eigen::Matrix3f Z = (J * J.transpose() + lambda * lambda * Eigen::Matrix3f::Identity()).inverse();
        Eigen::Vector3f dq = J.transpose() * Z * e;

        q.coxa += HexapodUtils::radiansToDegrees(dq[0]);
        q.femur += HexapodUtils::radiansToDegrees(dq[1]);
        q.tibia += HexapodUtils::radiansToDegrees(dq[2]);

        if (params.ik.clamp_joints) // option taken from Open SHC
        {
            q.coxa = constrainAngle(q.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
            q.femur = constrainAngle(q.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
            q.tibia = constrainAngle(q.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
        }
    }

    last_error = KINEMATICS_ERROR; // did not converge
    return q;
}

// Forward kinematics using DH transforms
Point3D HexapodLocomotionSystem::calculateForwardKinematics(int leg_index, const JointAngles &angles) {
    // Compute total leg transform
    Eigen::Matrix4f transform = calculateLegTransform(leg_index, angles);

    // Extract foot position
    Point3D position;
    position.x = transform(0, 3);
    position.y = transform(1, 3);
    position.z = transform(2, 3);

    return position;
}

// DH transformation calculation
Eigen::Matrix4f HexapodLocomotionSystem::calculateDHTransform(float a, float alpha, float d, float theta) {
    float cos_theta = cos(HexapodUtils::degreesToRadians(theta));
    float sin_theta = sin(HexapodUtils::degreesToRadians(theta));
    float cos_alpha = cos(HexapodUtils::degreesToRadians(alpha));
    float sin_alpha = sin(HexapodUtils::degreesToRadians(alpha));

    Eigen::Matrix4f transform;
    transform << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta,
        sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta,
        0, sin_alpha, cos_alpha, d,
        0, 0, 0, 1;

    return transform;
}

// Complete leg transform
Eigen::Matrix4f HexapodLocomotionSystem::calculateLegTransform(int leg_index,
                                                               const JointAngles &q) {
    // Base transformation (hexagon)
    const float base_angle_deg = leg_index * 60.0f;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(base_angle_deg));
    T(1, 3) = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(base_angle_deg));
    T.block<3, 3>(0, 0) = HexapodUtils::rotationMatrixZ(base_angle_deg);

    // List of joint angles
    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Encadenar Aᵢ = A(aᵢ, αᵢ, dᵢ, θ₀ᵢ + θᵢ)
    for (int j = 0; j < DOF_PER_LEG; ++j) {
        float a = params.dh_parameters[leg_index][j][0];
        float alpha = params.dh_parameters[leg_index][j][1];
        float d = params.dh_parameters[leg_index][j][2];
        float theta0 = params.dh_parameters[leg_index][j][3];
        float theta = theta0 + joint_deg[j];

        T *= calculateDHTransform(a, alpha, d, theta);
    }
    return T;
}

// Jacobian calculation
Eigen::Matrix3f HexapodLocomotionSystem::calculateAnalyticJacobian(int leg, const JointAngles &q) {
    // Partial FK to obtain origins and Z axes for each joint
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    Eigen::Vector3f o[DOF_PER_LEG + 1]; // origins 0..3
    Eigen::Vector3f z[DOF_PER_LEG];     // ejes Z 0…2

    // Base transform
    const float base_angle = leg * 60.0f;
    T(0, 3) = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(base_angle));
    T(1, 3) = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(base_angle));
    T.block<3, 3>(0, 0) = HexapodUtils::rotationMatrixZ(base_angle);

    o[0] = T.block<3, 1>(0, 3);

    const float q_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 0; j < DOF_PER_LEG; ++j) {
        float a = params.dh_parameters[leg][j][0];
        float alpha = params.dh_parameters[leg][j][1];
        float d = params.dh_parameters[leg][j][2];
        float theta0 = params.dh_parameters[leg][j][3];
        float theta = theta0 + q_deg[j];

        // A_j
        T *= calculateDHTransform(a, alpha, d, theta);
        z[j] = T.block<3, 1>(0, 2);     // eje Z del joint j en global
        o[j + 1] = T.block<3, 1>(0, 3); // origen joint j+1
    }

    Eigen::Vector3f p_end = o[DOF_PER_LEG];
    Eigen::Matrix3f J;

    for (int j = 0; j < DOF_PER_LEG; ++j)
        J.col(j) = z[j].cross(p_end - o[j]);

    return J;
}

Eigen::MatrixXf HexapodLocomotionSystem::calculateJacobian(int leg, const JointAngles &q) {
    return calculateAnalyticJacobian(leg, q);
}

/* Transformar punto mundo → cuerpo = Rᵀ·(p - p₀) */
Point3D HexapodLocomotionSystem::transformWorldToBody(const Point3D &p_world) const {
    // Vector relativo al centro del cuerpo
    Point3D rel(p_world.x - body_position[0],
                p_world.y - body_position[1],
                p_world.z - body_position[2]);

    // Rotate with negative angles (inverse)
    Eigen::Vector3f neg_rpy(-body_orientation[0],
                            -body_orientation[1],
                            -body_orientation[2]);
    return HexapodUtils::rotatePoint(rel, neg_rpy);
}

/* Store angles both in RAM and servos */
bool HexapodLocomotionSystem::setLegJointAngles(int leg, const JointAngles &q) {
    if (!servo_interface)
        return false;

    joint_angles[leg] = q; // estado interno
    servo_interface->setJointAngle(leg, 0, q.coxa);
    servo_interface->setJointAngle(leg, 1, q.femur);
    servo_interface->setJointAngle(leg, 2, q.tibia);
    return true;
}

// Gait planner
bool HexapodLocomotionSystem::setGaitType(GaitType gait) {
    current_gait = gait;
    gait_phase = 0.0f;

    // Configure gait-specific parameters
    switch (gait) {
    case TRIPOD_GAIT:
        // Two alternating leg groups (0,2,4 and 1,3,5)
        // 50% of the cycle in stance, 50% in swing
        stance_duration = 0.5f;
        swing_duration = 0.5f;
        cycle_frequency = 2.0f; // Hz - fast for speed

        // Set phase offsets per leg
        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = (i % 2) * 0.5f; // Group A: 0, Group B: 0.5
        }

        Serial.println("Gait set: TRIPOD - fast, medium stability");
        break;

    case WAVE_GAIT:
        // One leg at a time in sequence (0->1->2->3->4->5)
        // 83.3% stance, 16.7% swing for maximum stability
        stance_duration = 0.833f;
        swing_duration = 0.167f;
        cycle_frequency = 1.0f; // Hz slower for stability

        // Uniform sequence with 1/6 cycle offset
        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = i / 6.0f;
        }

        Serial.println("Gait set: WAVE - slow, maximum stability");
        break;

    case RIPPLE_GAIT:
        // Tres grupos de dos patas: (0,3), (1,4), (2,5)
        // 66.7% apoyo, 33.3% vuelo - balance velocidad/estabilidad
        stance_duration = 0.667f;
        swing_duration = 0.333f;
        cycle_frequency = 1.5f; // Hz intermedio

        // Tres grupos con offset de 1/3 del ciclo
        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = (i / 2) / 3.0f; // Grupos: 0, 0, 1/3, 1/3, 2/3, 2/3
        }

        Serial.println("Gait configurado: RIPPLE - Velocidad media, buena estabilidad");
        break;

    case METACHRONAL_GAIT:
        // Gait ondulatorio para terreno irregular
        // Metachronal wave propagation around the hexagon
        stance_duration = 0.75f;
        swing_duration = 0.25f;
        cycle_frequency = 1.2f; // Hz adaptativo

        // Metachronal wave propagation with sinusoidal component
        for (int i = 0; i < NUM_LEGS; i++) {
            float base_offset = i / 6.0f;
            float wave_component = sin(i * M_PI / 3.0f) * 0.1f;
            leg_phase_offsets[i] = base_offset + wave_component;

            // Normalizar entre 0 y 1
            if (leg_phase_offsets[i] < 0)
                leg_phase_offsets[i] += 1.0f;
            if (leg_phase_offsets[i] >= 1.0f)
                leg_phase_offsets[i] -= 1.0f;
        }

        Serial.println("Gait configurado: METACHRONAL - Adaptativo, terreno irregular");
        break;

    case ADAPTIVE_GAIT:
        // Gait adapting using FSR and IMU sensors
        // Initial parameters modified dynamically
        stance_duration = 0.6f;
        swing_duration = 0.4f;
        cycle_frequency = 1.5f;

        // Fase inicial como TRIPOD, luego se adapta
        for (int i = 0; i < NUM_LEGS; i++) {
            leg_phase_offsets[i] = (i % 2) * 0.5f;
        }

        // Call adaptation function
        adaptGaitToTerrain();

        Serial.println("Gait set: ADAPTIVE - automatic based on sensors");
        break;

    default:
        Serial.println("Error: Tipo de gait no reconocido");
        return false;
    }

    // Update step parameters for the new gait
    updateStepParameters();

    // Reinicializar estados de las patas
    for (int i = 0; i < NUM_LEGS; i++) {
        leg_states[i] = STANCE_PHASE;
    }

    return true;
}

// Gait sequence planning
bool HexapodLocomotionSystem::planGaitSequence(float velocity_x, float velocity_y, float angular_velocity) {
    if (!system_enabled)
        return false;

    // Check limits de velocidad
    float total_velocity = sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
    if (total_velocity > params.max_velocity) {
        // Escalar velocidades
        float scale = params.max_velocity / total_velocity;
        velocity_x *= scale;
        velocity_y *= scale;
    }

    if (abs(angular_velocity) > params.max_angular_velocity) {
        angular_velocity = (angular_velocity > 0) ? params.max_angular_velocity : -params.max_angular_velocity;
    }

    // Calcular parameters de paso
    float cycle_time = 1.0f;                          // Tiempo de ciclo base (segundos)
    step_length = total_velocity * cycle_time * 0.5f; // 50% del desplazamiento por ciclo

    // Adjust step height according to speed
    step_height = 20.0f + (total_velocity / params.max_velocity) * 20.0f; // 20-40mm

    return true;
}

// Gait phase update
void HexapodLocomotionSystem::updateGaitPhase() {
    // float phase_increment = dt * params.control_frequency / 100.0f; //Original
    float phase_increment = dt * params.control_frequency;
    gait_phase += phase_increment;

    if (gait_phase >= 1.0f) {
        gait_phase -= 1.0f; // Reiniciar ciclo
    }

    updateLegStates();
}

// Foot trajectory calculation
Point3D HexapodLocomotionSystem::calculateFootTrajectory(int leg_index, float phase) {
    Point3D trajectory;

    // Base leg position
    float base_angle = leg_index * 60.0f;
    float base_x = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(base_angle)) + params.coxa_length;
    float base_y = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(base_angle));

    switch (current_gait) {
    case TRIPOD_GAIT: {
        // Grupo A: patas 0,2,4 - Grupo B: patas 1,3,5
        bool group_a = (leg_index % 2 == 0);
        float leg_phase = group_a ? phase : (phase + 0.5f);
        if (leg_phase >= 1.0f)
            leg_phase -= 1.0f;

        if (leg_phase < 0.5f) {
            // Fase de apoyo
            leg_states[leg_index] = STANCE_PHASE;
            float support_progress = leg_phase * 2.0f;
            trajectory.x = base_x + step_length * (0.5f - support_progress);
            trajectory.y = base_y;
            trajectory.z = -params.robot_height;
        } else {
            // Fase de vuelo
            leg_states[leg_index] = SWING_PHASE;
            float swing_progress = (leg_phase - 0.5f) * 2.0f;
            trajectory.x = base_x + step_length * (swing_progress - 0.5f);
            trajectory.y = base_y;
            // Parabolic height trajectory
            trajectory.z = -params.robot_height + step_height * sin(M_PI * swing_progress);
        }
        break;
    }

    case WAVE_GAIT: {
        // Cada pata tiene su fase offset
        float leg_phase = phase + (leg_index / 6.0f);
        if (leg_phase >= 1.0f)
            leg_phase -= 1.0f;

        if (leg_phase < 0.833f) { // 83.3% apoyo
            leg_states[leg_index] = STANCE_PHASE;
            float support_progress = leg_phase / 0.833f;
            trajectory.x = base_x + step_length * (0.5f - support_progress);
            trajectory.y = base_y;
            trajectory.z = -params.robot_height;
        } else { // 16.7% vuelo
            leg_states[leg_index] = SWING_PHASE;
            float swing_progress = (leg_phase - 0.833f) / 0.167f;
            trajectory.x = base_x + step_length * (swing_progress - 0.5f);
            trajectory.y = base_y;
            trajectory.z = -params.robot_height + step_height * sin(M_PI * swing_progress);
        }
        break;
    }

    case RIPPLE_GAIT: {
        // Gait ripple con 3 grupos de 2 patas
        int group = leg_index / 2;
        float leg_phase = phase + (group / 3.0f);
        if (leg_phase >= 1.0f)
            leg_phase -= 1.0f;

        if (leg_phase < 0.667f) { // 66.7% apoyo
            leg_states[leg_index] = STANCE_PHASE;
            float support_progress = leg_phase / 0.667f;
            trajectory.x = base_x + step_length * (0.5f - support_progress);
            trajectory.y = base_y;
            trajectory.z = -params.robot_height;
        } else { // 33.3% vuelo
            leg_states[leg_index] = SWING_PHASE;
            float swing_progress = (leg_phase - 0.667f) / 0.333f;
            trajectory.x = base_x + step_length * (swing_progress - 0.5f);
            trajectory.y = base_y;
            trajectory.z = -params.robot_height + step_height * sin(M_PI * swing_progress);
        }
        break;
    }

    case METACHRONAL_GAIT: {
        // Wave gait with metachronal propagation
        float leg_offset = leg_phase_offsets[leg_index];
        float leg_phase = phase + leg_offset;
        if (leg_phase >= 1.0f)
            leg_phase -= 1.0f;

        // 75% apoyo, 25% vuelo como configurado en setGaitType
        if (leg_phase < 0.75f) {
            leg_states[leg_index] = STANCE_PHASE;
            float support_progress = leg_phase / 0.75f;

            // Wavy trajectory during stance for terrain adaptation
            float wave_amplitude = step_length * 0.1f; // 10% de amplitud ondulada
            float wave_offset = sin(leg_phase * 2.0f * M_PI + leg_index * M_PI / 3.0f) * wave_amplitude;

            trajectory.x = base_x + step_length * (0.5f - support_progress) + wave_offset;
            trajectory.y = base_y + wave_offset * 0.5f; // Componente lateral menor
            trajectory.z = -params.robot_height;
        } else {
            leg_states[leg_index] = SWING_PHASE;
            float swing_progress = (leg_phase - 0.75f) / 0.25f;

            // Trayectoria de vuelo adaptativa con mayor altura para terreno irregular
            float adaptive_height = step_height * (1.2f + 0.3f * sin(leg_index * M_PI / 3.0f));

            trajectory.x = base_x + step_length * (swing_progress - 0.5f);
            trajectory.y = base_y;
            // Modified parabolic trajectory for extra clearance
            trajectory.z = -params.robot_height + adaptive_height * sin(M_PI * swing_progress) *
                                                      (1.0f + 0.2f * sin(2.0f * M_PI * swing_progress));
        }
        break;
    }

    case ADAPTIVE_GAIT: {
        // Adaptive gait based on FSR and IMU sensors
        float leg_phase = phase + leg_phase_offsets[leg_index];
        if (leg_phase >= 1.0f)
            leg_phase -= 1.0f;

        // Get sensor data for adaptation
        FSRData fsr_data = fsr_interface->readFSR(leg_index);
        IMUData imu_data = imu_interface->readIMU();

        // Adapt phase duration based on pressure and tilt
        float adaptive_stance_duration = stance_duration;
        float adaptive_swing_duration = swing_duration;

        if (imu_data.is_valid) {
            float total_tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);
            if (total_tilt > 10.0f) {
                // En terreno inclinado, aumentar tiempo de apoyo
                adaptive_stance_duration = min(0.8f, stance_duration + 0.1f);
                adaptive_swing_duration = 1.0f - adaptive_stance_duration;
            }
        }

        if (fsr_data.pressure > params.fsr_max_pressure * 0.7f) {
            // If pressure is high reduce swing time
            adaptive_stance_duration = min(0.85f, stance_duration + 0.05f);
            adaptive_swing_duration = 1.0f - adaptive_stance_duration;
        }

        if (leg_phase < adaptive_stance_duration) {
            leg_states[leg_index] = STANCE_PHASE;
            float support_progress = leg_phase / adaptive_stance_duration;

            // Slope compensation
            float tilt_compensation_x = 0.0f;
            float tilt_compensation_y = 0.0f;

            if (imu_data.is_valid) {
                tilt_compensation_x = -imu_data.pitch * 0.5f; // Compensar pitch
                tilt_compensation_y = -imu_data.roll * 0.5f;  // Compensar roll
            }

            trajectory.x = base_x + step_length * (0.5f - support_progress) + tilt_compensation_x;
            trajectory.y = base_y + tilt_compensation_y;
            trajectory.z = -params.robot_height;
        } else {
            leg_states[leg_index] = SWING_PHASE;
            float swing_progress = (leg_phase - adaptive_stance_duration) / adaptive_swing_duration;

            // Adaptive height depending on terrain
            float adaptive_step_height = step_height;

            // If irregular terrain detected (by FSR variation)
            float avg_pressure = 0.0f;
            int contact_legs = 0;
            for (int i = 0; i < NUM_LEGS; i++) {
                if (i != leg_index && leg_states[i] == STANCE_PHASE) {
                    FSRData other_fsr = fsr_interface->readFSR(i);
                    if (other_fsr.in_contact) {
                        avg_pressure += other_fsr.pressure;
                        contact_legs++;
                    }
                }
            }

            if (contact_legs > 0) {
                avg_pressure /= contact_legs;
                float pressure_variance = abs(fsr_data.pressure - avg_pressure);
                if (pressure_variance > params.fsr_max_pressure * 0.2f) {
                    // Terreno irregular detectado, aumentar altura de paso
                    adaptive_step_height *= 1.5f;
                }
            }

            // Trayectoria de vuelo adaptativa
            trajectory.x = base_x + step_length * (swing_progress - 0.5f);
            trajectory.y = base_y;

            // Parabolic trajectory with optional double peak for obstacles
            float base_height = adaptive_step_height * sin(M_PI * swing_progress);
            if (swing_progress > 0.3f && swing_progress < 0.7f) {
                // Additional mid-peak to clear obstacles
                base_height += adaptive_step_height * 0.3f * sin(5.0f * M_PI * swing_progress);
            }

            trajectory.z = -params.robot_height + base_height;
        }
        break;
    }

    default:
        // Static pose
        leg_states[leg_index] = STANCE_PHASE;
        trajectory.x = base_x;
        trajectory.y = base_y;
        trajectory.z = -params.robot_height;
        break;
    }

    return trajectory;
}

// Forward locomotion control
bool HexapodLocomotionSystem::walkForward(float velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(velocity, 0.0f, 0.0f);
}

// Backward locomotion control
bool HexapodLocomotionSystem::walkBackward(float velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(-velocity, 0.0f, 0.0f);
}

// In-place turning control
bool HexapodLocomotionSystem::turnInPlace(float angular_velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(0.0f, 0.0f, angular_velocity);
}

// Sideways locomotion control
bool HexapodLocomotionSystem::walkSideways(float velocity, bool right_direction) {
    if (!system_enabled)
        return false;

    float lateral_velocity = right_direction ? velocity : -velocity;
    return planGaitSequence(0.0f, lateral_velocity, 0.0f);
}

// Advance for “duration” seconds
bool HexapodLocomotionSystem::walkForward(float velocity, float duration) {
    if (!system_enabled)
        return false;

    // Plan gait to move forward along X (velocity m/s)
    planGaitSequence(velocity, 0.0f, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    // Blocking loop: run update() until duration elapsed
    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1) calls
        // delay(1);
    }

    stopMovement();
    return true;
}

// Move backward for “duration” seconds
bool HexapodLocomotionSystem::walkBackward(float velocity, float duration) {
    if (!system_enabled)
        return false;

    // Plan gait to move in -X (backwards)
    planGaitSequence(-velocity, 0.0f, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Turn in place for “duration” seconds
bool HexapodLocomotionSystem::turnInPlace(float angular_velocity, float duration) {
    if (!system_enabled)
        return false;

    // Plan gait with only angular component
    planGaitSequence(0.0f, 0.0f, angular_velocity);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Walk sideways (right/left) for “duration” seconds
bool HexapodLocomotionSystem::walkSideways(float velocity, float duration, bool right_direction) {
    if (!system_enabled)
        return false;

    float lateral_velocity = right_direction ? velocity : -velocity;
    planGaitSequence(0.0f, lateral_velocity, 0.0f);

    unsigned long startTime = millis();
    unsigned long durationMs = (unsigned long)(duration * 1000.0f);

    while (millis() - startTime < durationMs) {
        update();
        // In a real Arduino environment there may be delay(1)
        // delay(1);
    }

    stopMovement();
    return true;
}

// Stop movement while keeping current pose
bool HexapodLocomotionSystem::stopMovement() {
    if (!system_enabled)
        return false;

    // Plan gait with zero velocities to stop the robot
    planGaitSequence(0.0f, 0.0f, 0.0f);
    // Call update() once to resend no movement angles
    update();
    return true;
}

// Orientation control
bool HexapodLocomotionSystem::maintainOrientation(const Eigen::Vector3f &target_rpy) {
    if (!system_enabled || !imu_interface || !params.body_comp.enable)
        return false;

    IMUData imu = imu_interface->readIMU();
    if (!imu.is_valid) {
        last_error = IMU_ERROR;
        return false;
    }

    Eigen::Vector3f current(imu.roll, imu.pitch, imu.yaw);
    Eigen::Vector3f err = target_rpy - current;
    for (int i = 0; i < 3; ++i)
        err[i] = HexapodUtils::normalizeAngle(err[i]);

    // Aplica control P y filtra (Butterworth 1-pole)
    Eigen::Vector3f desired = body_orientation + err * params.body_comp.kp;
    body_orientation = body_orientation * (1.0f - params.body_comp.lp_alpha) + desired * params.body_comp.lp_alpha;

    // If tilt exceeds the threshold compensation is disabled
    if (fabsf(current[0]) > params.body_comp.max_tilt_deg ||
        fabsf(current[1]) > params.body_comp.max_tilt_deg)
        return false;

    // Maintain height (z) and reproject feet onto the ground
    body_position[2] = params.robot_height;
    reprojectStandingFeet();

    return true;
}

void HexapodLocomotionSystem::reprojectStandingFeet() {
    for (int leg = 0; leg < NUM_LEGS; ++leg) {
        if (leg_states[leg] != STANCE_PHASE)
            continue;

        // Current foot position world -> body
        Point3D tip_body = transformWorldToBody(leg_positions[leg]);

        // IK para la nueva actitud del cuerpo
        JointAngles q_new = calculateInverseKinematics(leg, tip_body);

        // Apply angles to servos and RAM
        setLegJointAngles(leg, q_new);

        // Update world position for consistency
        leg_positions[leg] = calculateForwardKinematics(leg, q_new);
    }
}

// Automatic tilt correction
bool HexapodLocomotionSystem::correctBodyTilt() {
    Eigen::Vector3f target_orientation(0.0f, 0.0f, body_orientation[2]); // Mantener yaw, corregir roll/pitch
    return maintainOrientation(target_orientation);
}

// Calculate orientation error
Eigen::Vector3f HexapodLocomotionSystem::calculateOrientationError() {
    if (!imu_interface)
        return Eigen::Vector3f::Zero();

    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return Eigen::Vector3f::Zero();

    Eigen::Vector3f current_orientation(imu_data.roll, imu_data.pitch, imu_data.yaw);
    return body_orientation - current_orientation;
}

// Verificar margen de estabilidad
bool HexapodLocomotionSystem::checkStabilityMargin() {
    if (!system_enabled)
        return false;

    // Compute support polygon
    std::vector<Point3D> support_points;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_states[i] == STANCE_PHASE) {
            FSRData fsr_data = fsr_interface->readFSR(i);
            if (fsr_data.in_contact) {
                support_points.push_back(leg_positions[i]);
            }
        }
    }

    // Necesitamos al menos 3 puntos para estabilidad
    if (support_points.size() < 3) {
        last_error = STABILITY_ERROR;
        return false;
    }

    // Compute center of pressure
    Eigen::Vector2f cop = calculateCenterOfPressure();

    // Check if COP is inside the support polygon
    // Simplified implementation: check minimum distance to edges
    float min_distance = params.stability_margin + 10.0f; // Margen adicional

    for (size_t i = 0; i < support_points.size(); i++) {
        Point3D p1 = support_points[i];
        Point3D p2 = support_points[(i + 1) % support_points.size()];

        // Compute COP distance to line p1-p2
        float line_dist = abs((p2.y - p1.y) * cop[0] - (p2.x - p1.x) * cop[1] + p2.x * p1.y - p2.y * p1.x) /
                          sqrt((p2.y - p1.y) * (p2.y - p1.y) + (p2.x - p1.x) * (p2.x - p1.x));

        if (line_dist < min_distance) {
            min_distance = line_dist;
        }
    }

    return min_distance >= params.stability_margin;
}

// Compute center of pressure
Eigen::Vector2f HexapodLocomotionSystem::calculateCenterOfPressure() {
    Eigen::Vector2f cop(0.0f, 0.0f);
    float total_force = 0.0f;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_states[i] == STANCE_PHASE) {
            FSRData fsr_data = fsr_interface->readFSR(i);
            if (fsr_data.in_contact && fsr_data.pressure > 0) {
                cop[0] += leg_positions[i].x * fsr_data.pressure;
                cop[1] += leg_positions[i].y * fsr_data.pressure;
                total_force += fsr_data.pressure;
            }
        }
    }

    if (total_force > 0) {
        cop /= total_force;
    }

    return cop;
}

// Compute stability index
float HexapodLocomotionSystem::calculateStabilityIndex() {
    if (!checkStabilityMargin())
        return 0.0f;

    Eigen::Vector2f cop = calculateCenterOfPressure();
    float stability_index = 1.0f;

    // Distance to closest edge of the support polygon
    // Simplified implementation
    float min_edge_distance = 1000.0f;

    for (int i = 0; i < NUM_LEGS; i++) {
        if (leg_states[i] == STANCE_PHASE) {
            float distance = sqrt((leg_positions[i].x - cop[0]) * (leg_positions[i].x - cop[0]) +
                                  (leg_positions[i].y - cop[1]) * (leg_positions[i].y - cop[1]));
            if (distance < min_edge_distance) {
                min_edge_distance = distance;
            }
        }
    }

    // Normalize index (0-1)
    stability_index = min_edge_distance / (params.stability_margin * 3.0f);
    return min(1.0f, max(0.0f, stability_index));
}

// Check static stability
bool HexapodLocomotionSystem::isStaticallyStable() {
    return calculateStabilityIndex() > 0.2f; // Minimum stability threshold
}

// Control de pose del cuerpo
bool HexapodLocomotionSystem::setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation) {
    if (!system_enabled)
        return false;

    // For each leg take current world position and convert it
    for (int i = 0; i < NUM_LEGS; ++i) {

        // Get old leg relative position (old body frame)
        Point3D old_leg_body = transformWorldToBody(leg_positions[i]);

        // Rotate that position with the NEW orientation
        Point3D new_leg_body = HexapodUtils::rotatePoint(old_leg_body, orientation);

        // Convert back to world coordinates with the NEW body position
        Point3D new_leg_world;
        new_leg_world.x = position[0] + new_leg_body.x;
        new_leg_world.y = position[1] + new_leg_body.y;
        new_leg_world.z = position[2] + new_leg_body.z;

        // Realizar IK para ubicar la pata en new_leg_body (coordenadas relativas a cuerpo)
        JointAngles angles = calculateInverseKinematics(i, new_leg_body);
        if (!checkJointLimits(i, angles)) {
            last_error = KINEMATICS_ERROR;
            return false;
        }
        setLegJointAngles(i, angles);
        leg_positions[i] = new_leg_world;
    }

    // Finalmente, actualizar pose del cuerpo
    body_position = position;
    body_orientation = orientation;
    return true;
}

// Establecer pose de pie
bool HexapodLocomotionSystem::setStandingPose() {
    Eigen::Vector3f standing_position(0.0f, 0.0f, params.robot_height);
    Eigen::Vector3f neutral_orientation(0.0f, 0.0f, 0.0f);

    // Calcular posiciones por defecto de las patas
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle = i * 60.0f;
        float leg_reach = params.coxa_length + params.femur_length * 0.7f + params.tibia_length * 0.5f;

        leg_positions[i].x = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(angle)) +
                             leg_reach * cos(HexapodUtils::degreesToRadians(angle));
        leg_positions[i].y = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(angle)) +
                             leg_reach * sin(HexapodUtils::degreesToRadians(angle));
        leg_positions[i].z = -params.robot_height;

        leg_states[i] = STANCE_PHASE;
    }

    return setBodyPose(standing_position, neutral_orientation);
}

// Establecer pose agachada
bool HexapodLocomotionSystem::setCrouchPose() {
    Eigen::Vector3f crouch_position(0.0f, 0.0f, params.robot_height * 0.6f);
    Eigen::Vector3f neutral_orientation(0.0f, 0.0f, 0.0f);

    // Acercar las patas al centro
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle = i * 60.0f;
        float leg_reach = params.coxa_length + params.femur_length * 0.5f;

        leg_positions[i].x = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(angle)) +
                             leg_reach * cos(HexapodUtils::degreesToRadians(angle));
        leg_positions[i].y = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(angle)) +
                             leg_reach * sin(HexapodUtils::degreesToRadians(angle));
        leg_positions[i].z = -params.robot_height * 0.6f;

        leg_states[i] = STANCE_PHASE;
    }

    return setBodyPose(crouch_position, neutral_orientation);
}

// Main system update
bool HexapodLocomotionSystem::update() {
    if (!system_enabled)
        return false;

    unsigned long current_time = millis();
    dt = (current_time - last_update_time) / 1000.0f; // Convertir a segundos
    last_update_time = current_time;

    // Limitar dt para evitar saltos grandes
    if (dt > 0.1f)
        dt = 0.1f;

    // Actualizar fase de marcha
    updateGaitPhase();

    // Calcular nuevas posiciones de las patas
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D target_position = calculateFootTrajectory(i, gait_phase);

        // Compute inverse kinematics
        JointAngles target_angles = calculateInverseKinematics(i, target_position);

        // Check limits
        if (checkJointLimits(i, target_angles)) {
            joint_angles[i] = target_angles;
            leg_positions[i] = target_position;

            // Send commands to servos
            for (int j = 0; j < DOF_PER_LEG; j++) {
                float angle_value;
                switch (j) {
                case 0:
                    angle_value = target_angles.coxa;
                    break;
                case 1:
                    angle_value = target_angles.femur;
                    break;
                case 2:
                    angle_value = target_angles.tibia;
                    break;
                }
                servo_interface->setJointAngle(i, j, angle_value);
            }
        }
    }

    // Automatic orientation control
    if (imu_interface && imu_interface->isConnected()) {
        correctBodyTilt();
    }

    // Verificar estabilidad
    if (!checkStabilityMargin()) {
        // Implementar acciones correctivas si es necesario
        last_error = STABILITY_ERROR;
    }

    return true;
}

// Error handling
String HexapodLocomotionSystem::getErrorMessage(ErrorCode error) {
    switch (error) {
    case NO_ERROR:
        return "Sin errores";
    case IMU_ERROR:
        return "Error en IMU";
    case FSR_ERROR:
        return "Error en sensores FSR";
    case SERVO_ERROR:
        return "Error en servos";
    case KINEMATICS_ERROR:
        return "Kinematics error";
    case STABILITY_ERROR:
        return "Error de estabilidad";
    case PARAMETER_ERROR:
        return "Error en parameters";
    default:
        return "Error desconocido";
    }
}

bool HexapodLocomotionSystem::handleError(ErrorCode error) {
    last_error = error;

    switch (error) {
    case IMU_ERROR:
        // Try to reinitialize IMU
        if (imu_interface) {
            return imu_interface->initialize();
        }
        break;

    case FSR_ERROR:
        // Try to recalibrate FSRs
        if (fsr_interface) {
            for (int i = 0; i < NUM_LEGS; i++) {
                fsr_interface->calibrateFSR(i);
            }
            return true;
        }
        break;

    case SERVO_ERROR:
        // Try to reinitialize servos
        if (servo_interface) {
            return servo_interface->initialize();
        }
        break;

    case STABILITY_ERROR:
        // Adopt a more stable pose
        return setCrouchPose();

    case KINEMATICS_ERROR:
        // Return to a safe pose
        return setStandingPose();

    default:
        return false;
    }

    return false;
}

// System self test
bool HexapodLocomotionSystem::performSelfTest() {
    Serial.println("=== Starting self test ===");

    // IMU test
    if (!imu_interface || !imu_interface->isConnected()) {
        Serial.println("X Error: IMU no conectado");
        return false;
    }

    IMUData imu_test = imu_interface->readIMU();
    if (!imu_test.is_valid) {
        Serial.println("X Error: invalid IMU data");
        return false;
    }
    Serial.println("OK IMU funcionando correctamente");

    // FSR test
    for (int i = 0; i < NUM_LEGS; i++) {
        FSRData fsr_test = fsr_interface->readFSR(i);
        if (fsr_test.pressure < 0) {
            Serial.print("X Error: FSR pata ");
            Serial.print(i);
            Serial.println(" mal funcionamiento");
            return false;
        }
    }
    Serial.println("OK Todos los FSRs funcionando");

    // Servo test
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < DOF_PER_LEG; j++) {
            float current_angle = servo_interface->getJointAngle(i, j);
            if (current_angle < -180 || current_angle > 180) {
                Serial.print("X Error: Servo pata ");
                Serial.print(i);
                Serial.print(" joint ");
                Serial.println(j);
                return false;
            }
        }
    }
    Serial.println("OK Todos los servos funcionando");

    // Kinematics test
    for (int i = 0; i < NUM_LEGS; i++) {
        Point3D test_point(100, 0, -100);
        JointAngles angles = calculateInverseKinematics(i, test_point);
        Point3D calculated_point = calculateForwardKinematics(i, angles);

        float error = HexapodUtils::distance3D(test_point, calculated_point);
        if (error > 5.0f) { // Error greater than 5mm
            Serial.print("X Error: leg kinematics ");
            Serial.print(i);
            Serial.print(" error=");
            Serial.print(error);
            Serial.println("mm");
            return false;
        }
    }
    Serial.println("OK Kinematics working correctly");

    Serial.println("=== Auto-diagnostic completado exitosamente ===");
    return true;
}

// Helper functions
void HexapodLocomotionSystem::initializeDefaultPose() {
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle = i * 60.0f;
        leg_positions[i].x = params.hexagon_radius * cos(HexapodUtils::degreesToRadians(angle)) + params.coxa_length;
        leg_positions[i].y = params.hexagon_radius * sin(HexapodUtils::degreesToRadians(angle));
        leg_positions[i].z = -params.robot_height;

        joint_angles[i] = JointAngles(0, 45, -90); // Default angles
        leg_states[i] = STANCE_PHASE;
    }
}

void HexapodLocomotionSystem::updateLegStates() {
    // This function is called from calculateFootTrajectory
    // Leg states are updated there according to the current gait
}

void HexapodLocomotionSystem::updateStepParameters() {
    // Adjust step parameters depending on gait type
    switch (current_gait) {
    case TRIPOD_GAIT:
        // Longer steps for speed
        step_length = 60.0f; // mm
        step_height = 35.0f; // mm
        break;

    case WAVE_GAIT:
        // Shorter steps for stability
        step_length = 40.0f; // mm
        step_height = 25.0f; // mm
        break;

    case RIPPLE_GAIT:
        // Medium steps for balance
        step_length = 50.0f; // mm
        step_height = 30.0f; // mm
        break;

    case METACHRONAL_GAIT:
        // Adaptive steps
        step_length = 45.0f; // mm
        step_height = 30.0f; // mm
        break;

    case ADAPTIVE_GAIT:
        // They will be adjusted dynamically
        step_length = 50.0f; // mm inicial
        step_height = 30.0f; // mm inicial
        break;
    }

    // Verify that parameters are within limits
    step_length = constrainAngle(step_length, 20.0f, 80.0f);
    step_height = constrainAngle(step_height, 15.0f, 50.0f);
}

bool HexapodLocomotionSystem::checkJointLimits(int leg_index, const JointAngles &angles) {
    return (angles.coxa >= params.coxa_angle_limits[0] && angles.coxa <= params.coxa_angle_limits[1] &&
            angles.femur >= params.femur_angle_limits[0] && angles.femur <= params.femur_angle_limits[1] &&
            angles.tibia >= params.tibia_angle_limits[0] && angles.tibia <= params.tibia_angle_limits[1]);
}

float HexapodLocomotionSystem::constrainAngle(float angle, float min_angle, float max_angle) {
    return max(min_angle, min(max_angle, angle));
}

bool HexapodLocomotionSystem::validateParameters() {
    return (params.hexagon_radius > 0 && params.coxa_length > 0 &&
            params.femur_length > 0 && params.tibia_length > 0 &&
            params.robot_height > 0 && params.control_frequency > 0);
}

void HexapodLocomotionSystem::adaptGaitToTerrain() {
    // Analyze FSR data to adapt gait
    float avg_pressure = 0;
    int contact_count = 0;

    for (int i = 0; i < NUM_LEGS; i++) {
        FSRData fsr_data = fsr_interface->readFSR(i);
        if (fsr_data.in_contact) {
            avg_pressure += fsr_data.pressure;
            contact_count++;
        }
    }

    if (contact_count > 0) {
        avg_pressure /= contact_count;

        // If average pressure is high use a more stable gait
        if (avg_pressure > params.fsr_max_pressure * 0.8f) {
            current_gait = WAVE_GAIT;
        } else {
            current_gait = TRIPOD_GAIT;
        }
    }
}

// Additional implementations pending

bool HexapodLocomotionSystem::setLegPosition(int leg_index, const Point3D &position) {
    if (!system_enabled || leg_index < 0 || leg_index >= NUM_LEGS)
        return false;

    // Calculate inverse kinematics for the new position
    JointAngles angles = calculateInverseKinematics(leg_index, position);

    if (!checkJointLimits(leg_index, angles)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }

    // Update position and angles
    leg_positions[leg_index] = position;
    joint_angles[leg_index] = angles;

    // Send commands to servos
    for (int j = 0; j < DOF_PER_LEG; j++) {
        float angle_value;
        switch (j) {
        case 0:
            angle_value = angles.coxa;
            break;
        case 1:
            angle_value = angles.femur;
            break;
        case 2:
            angle_value = angles.tibia;
            break;
        }
        servo_interface->setJointAngle(leg_index, j, angle_value);
    }

    return true;
}

bool HexapodLocomotionSystem::setStepParameters(float height, float length) {
    if (height < 15.0f || height > 50.0f || length < 20.0f || length > 80.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    step_height = height;
    step_length = length;
    return true;
}

bool HexapodLocomotionSystem::setParameters(const HexapodParameters &new_params) {
    // Validar nuevos parameters
    if (new_params.hexagon_radius <= 0 || new_params.coxa_length <= 0 ||
        new_params.femur_length <= 0 || new_params.tibia_length <= 0) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params = new_params;
    return validateParameters();
}

bool HexapodLocomotionSystem::setControlFrequency(float frequency) {
    if (frequency < 10.0f || frequency > 200.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params.control_frequency = frequency;
    return true;
}

void HexapodLocomotionSystem::printSystemStatus() {
    Serial.println("=== Hexapod System Status ===");
    Serial.print("Sistema habilitado: ");
    Serial.println(system_enabled ? "Yes" : "No");
    Serial.print("Gait actual: ");
    Serial.println(current_gait);
    Serial.print("Fase de gait: ");
    Serial.println(gait_phase, 3);
    Serial.print("Índice de estabilidad: ");
    Serial.println(calculateStabilityIndex(), 3);
    Serial.print("Último error: ");
    Serial.println(getErrorMessage(last_error));
    Serial.println("==================================");
}

void HexapodLocomotionSystem::printLegStatus(int leg_index) {
    if (leg_index < 0 || leg_index >= NUM_LEGS)
        return;

    Serial.print("=== Estado Pata ");
    Serial.print(leg_index);
    Serial.println(" ===");
    Serial.print("Estado: ");
    Serial.println(leg_states[leg_index]);
    Serial.print("Position: (");
    Serial.print(leg_positions[leg_index].x, 1);
    Serial.print(", ");
    Serial.print(leg_positions[leg_index].y, 1);
    Serial.print(", ");
    Serial.print(leg_positions[leg_index].z, 1);
    Serial.println(")");
    Serial.print("Ángulos: (");
    Serial.print(joint_angles[leg_index].coxa, 1);
    Serial.print(", ");
    Serial.print(joint_angles[leg_index].femur, 1);
    Serial.print(", ");
    Serial.print(joint_angles[leg_index].tibia, 1);
    Serial.println(")");

    FSRData fsr_data = fsr_interface->readFSR(leg_index);
    Serial.print("FSR - Pressure: ");
    Serial.print(fsr_data.pressure, 2);
    Serial.print(" kg, Contacto: ");
    Serial.println(fsr_data.in_contact ? "Yes" : "No");
    Serial.println("===================");
}

float HexapodLocomotionSystem::calculateLegReach(int leg_index) {
    return params.coxa_length + params.femur_length + params.tibia_length;
}

void HexapodLocomotionSystem::adjustStepParameters() {
    // Adjust parameters according to terrain conditions
    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return;

    // If slope is large reduce step size
    float total_tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);
    if (total_tilt > 15.0f) {
        step_height *= 0.8f;
        step_length *= 0.7f;
    }

    // Limitar parameters
    step_height = constrainAngle(step_height, 15.0f, 50.0f);
    step_length = constrainAngle(step_length, 20.0f, 80.0f);
}

void HexapodLocomotionSystem::compensateForSlope() {
    if (!imu_interface)
        return;

    IMUData imu_data = imu_interface->readIMU();
    if (!imu_data.is_valid)
        return;

    // Compensate tilt by adjusting body position
    float roll_compensation = -imu_data.roll * 0.5f; // Compensation factor
    float pitch_compensation = -imu_data.pitch * 0.5f;

    // Adjust body orientation
    body_orientation[0] += roll_compensation * dt;
    body_orientation[1] += pitch_compensation * dt;

    // Clamp compensation
    body_orientation[0] = constrainAngle(body_orientation[0], -15.0f, 15.0f);
    body_orientation[1] = constrainAngle(body_orientation[1], -15.0f, 15.0f);
}
