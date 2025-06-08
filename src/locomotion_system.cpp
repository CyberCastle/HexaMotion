/**
 * @file locomotion_system.cpp
 * @brief Implementation of the Locomotion Control System
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

#include "locomotion_system.h"
#include "math_utils.h"
#include <vector>
#include <algorithm>

// Constructor
LocomotionSystem::LocomotionSystem(const Parameters &params)
    : params(params), imu_interface(nullptr), fsr_interface(nullptr), servo_interface(nullptr),
      current_gait(TRIPOD_GAIT), gait_phase(0.0f), step_height(30.0f), step_length(50.0f),
      stance_duration(0.5f), swing_duration(0.5f), cycle_frequency(2.0f),
      system_enabled(false), last_update_time(0), dt(0.02f), last_error(NO_ERROR),
      model(params), pose_ctrl(nullptr), walk_ctrl(nullptr), admittance_ctrl(nullptr) {

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
LocomotionSystem::~LocomotionSystem() {
    system_enabled = false;
    delete pose_ctrl;
    delete walk_ctrl;
    delete admittance_ctrl;
}

// System initialization
bool LocomotionSystem::initialize(IIMUInterface *imu, IFSRInterface *fsr, IServoInterface *servo) {
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

    pose_ctrl = new PoseController(model, servo_interface);
    walk_ctrl = new WalkController(model);
    admittance_ctrl = new AdmittanceController(model, imu_interface, fsr_interface);

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
bool LocomotionSystem::calibrateSystem() {
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
JointAngles LocomotionSystem::calculateInverseKinematics(int leg,
                                                        const Point3D &p_target) {
    return model.inverseKinematics(leg, p_target);
}

// Forward kinematics using DH transforms
Point3D LocomotionSystem::calculateForwardKinematics(int leg_index, const JointAngles &angles) {
    return model.forwardKinematics(leg_index, angles);
}

// DH transformation calculation
Eigen::Matrix4f LocomotionSystem::calculateDHTransform(float a, float alpha, float d, float theta) {
    return math_utils::dhTransform(a, alpha, d, theta);
}

// Complete leg transform
Eigen::Matrix4f LocomotionSystem::calculateLegTransform(int leg_index,
                                                       const JointAngles &q) {
    return model.legTransform(leg_index, q);
}

// Jacobian calculation
Eigen::Matrix3f LocomotionSystem::calculateAnalyticJacobian(int leg, const JointAngles &q) {
    return model.analyticJacobian(leg, q);
}

Eigen::MatrixXf LocomotionSystem::calculateJacobian(int leg, const JointAngles &q) {
    return model.analyticJacobian(leg, q);
}

/* Transformar punto mundo → cuerpo = Rᵀ·(p - p₀) */
Point3D LocomotionSystem::transformWorldToBody(const Point3D &p_world) const {
    // Vector relativo al centro del cuerpo
    Point3D rel(p_world.x - body_position[0],
                p_world.y - body_position[1],
                p_world.z - body_position[2]);

    // Rotate with negative angles (inverse)
    Eigen::Vector3f neg_rpy(-body_orientation[0],
                            -body_orientation[1],
                            -body_orientation[2]);
    return math_utils::rotatePoint(rel, neg_rpy);
}

/* Store angles both in RAM and servos */
bool LocomotionSystem::setLegJointAngles(int leg, const JointAngles &q) {
    if (!servo_interface)
        return false;

    joint_angles[leg] = q; // estado interno
    servo_interface->setJointAngle(leg, 0, q.coxa);
    servo_interface->setJointAngle(leg, 1, q.femur);
    servo_interface->setJointAngle(leg, 2, q.tibia);
    return true;
}

// Gait planner
bool LocomotionSystem::setGaitType(GaitType gait) {
    if (!walk_ctrl) return false;
    current_gait = gait;
    gait_phase = 0.0f;
    return walk_ctrl->setGaitType(gait);
}

// Gait sequence planning
bool LocomotionSystem::planGaitSequence(float vx, float vy, float omega) {
    if (!walk_ctrl) return false;
    return walk_ctrl->planGaitSequence(vx, vy, omega);
}

// Gait phase update
void LocomotionSystem::updateGaitPhase() {
    if (walk_ctrl)
        walk_ctrl->updateGaitPhase(dt);
}

// Foot trajectory calculation
Point3D LocomotionSystem::calculateFootTrajectory(int leg_index, float phase) {
    if (!walk_ctrl)
        return Point3D();
    return walk_ctrl->footTrajectory(leg_index, phase, step_height, step_length,
                                     stance_duration, swing_duration, params.robot_height,
                                     leg_phase_offsets, leg_states, fsr_interface, imu_interface);
}


// Forward locomotion control
bool LocomotionSystem::walkForward(float velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(velocity, 0.0f, 0.0f);
}

// Backward locomotion control
bool LocomotionSystem::walkBackward(float velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(-velocity, 0.0f, 0.0f);
}

// In-place turning control
bool LocomotionSystem::turnInPlace(float angular_velocity) {
    if (!system_enabled)
        return false;

    return planGaitSequence(0.0f, 0.0f, angular_velocity);
}

// Sideways locomotion control
bool LocomotionSystem::walkSideways(float velocity, bool right_direction) {
    if (!system_enabled)
        return false;

    float lateral_velocity = right_direction ? velocity : -velocity;
    return planGaitSequence(0.0f, lateral_velocity, 0.0f);
}

// Advance for “duration” seconds
bool LocomotionSystem::walkForward(float velocity, float duration) {
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
bool LocomotionSystem::walkBackward(float velocity, float duration) {
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
bool LocomotionSystem::turnInPlace(float angular_velocity, float duration) {
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
bool LocomotionSystem::walkSideways(float velocity, float duration, bool right_direction) {
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
bool LocomotionSystem::stopMovement() {
    if (!system_enabled)
        return false;

    // Plan gait with zero velocities to stop the robot
    planGaitSequence(0.0f, 0.0f, 0.0f);
    // Call update() once to resend no movement angles
    update();
    return true;
}

// Orientation control
bool LocomotionSystem::maintainOrientation(const Eigen::Vector3f &target_rpy) {
    if (!system_enabled || !admittance_ctrl)
        return false;
    return admittance_ctrl->maintainOrientation(target_rpy, body_orientation, dt);
}

void LocomotionSystem::reprojectStandingFeet() {
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
bool LocomotionSystem::correctBodyTilt() {
    Eigen::Vector3f target_orientation(0.0f, 0.0f, body_orientation[2]);
    return maintainOrientation(target_orientation);
}

// Calculate orientation error
Eigen::Vector3f LocomotionSystem::calculateOrientationError() {
    if (!admittance_ctrl)
        return Eigen::Vector3f::Zero();
    return admittance_ctrl->orientationError(body_orientation);
}

// Verificar margen de estabilidad
bool LocomotionSystem::checkStabilityMargin() {
    if (!system_enabled || !admittance_ctrl)
        return false;
    return admittance_ctrl->checkStability(leg_positions, leg_states);
}

// Compute center of pressure
Eigen::Vector2f LocomotionSystem::calculateCenterOfPressure() {
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
float LocomotionSystem::calculateStabilityIndex() {
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
    return std::min(1.0f, std::max(0.0f, stability_index));
}

// Check static stability
bool LocomotionSystem::isStaticallyStable() {
    return calculateStabilityIndex() > 0.2f; // Minimum stability threshold
}

// Control de pose del cuerpo
bool LocomotionSystem::setBodyPose(const Eigen::Vector3f &position, const Eigen::Vector3f &orientation) {
    if (!system_enabled || !pose_ctrl)
        return false;
    if (!pose_ctrl->setBodyPose(position, orientation, leg_positions, joint_angles)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }
    body_position = position;
    body_orientation = orientation;
    return true;
}

// Establecer pose de pie
bool LocomotionSystem::setStandingPose() {
    if (!pose_ctrl)
        return false;
    return pose_ctrl->setStandingPose(leg_positions, joint_angles, params.robot_height);
}

// Establecer pose agachada
bool LocomotionSystem::setCrouchPose() {
    if (!pose_ctrl)
        return false;
    return pose_ctrl->setCrouchPose(leg_positions, joint_angles, params.robot_height);
}

// Main system update
bool LocomotionSystem::update() {
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
String LocomotionSystem::getErrorMessage(ErrorCode error) {
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

bool LocomotionSystem::handleError(ErrorCode error) {
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
bool LocomotionSystem::performSelfTest() {
#if defined(ENABLE_SELF_TEST) && defined(ARDUINO)
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

        float error = math_utils::distance3D(test_point, calculated_point);
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
#else
    // Self test not available without Arduino environment
    return false;
#endif
}

// Helper functions
void LocomotionSystem::initializeDefaultPose() {
    for (int i = 0; i < NUM_LEGS; i++) {
        float angle = i * 60.0f;
        leg_positions[i].x = params.hexagon_radius * cos(math_utils::degreesToRadians(angle)) + params.coxa_length;
        leg_positions[i].y = params.hexagon_radius * sin(math_utils::degreesToRadians(angle));
        leg_positions[i].z = -params.robot_height;

        joint_angles[i] = JointAngles(0, 45, -90); // Default angles
        leg_states[i] = STANCE_PHASE;
    }
}

void LocomotionSystem::updateStepParameters() {
    // Calculate leg reach and robot dimensions
    float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float robot_height = params.robot_height;

    // Adjust step parameters depending on gait type using parametrizable factors
    switch (current_gait) {
    case TRIPOD_GAIT:
        // Longer steps for speed
        step_length = leg_reach * params.gait_factors.tripod_length_factor;
        step_height = robot_height * params.gait_factors.tripod_height_factor;
        break;

    case WAVE_GAIT:
        // Shorter steps for stability
        step_length = leg_reach * params.gait_factors.wave_length_factor;
        step_height = robot_height * params.gait_factors.wave_height_factor;
        break;

    case RIPPLE_GAIT:
        // Medium steps for balance
        step_length = leg_reach * params.gait_factors.ripple_length_factor;
        step_height = robot_height * params.gait_factors.ripple_height_factor;
        break;

    case METACHRONAL_GAIT:
        // Adaptive steps
        step_length = leg_reach * params.gait_factors.metachronal_length_factor;
        step_height = robot_height * params.gait_factors.metachronal_height_factor;
        break;

    case ADAPTIVE_GAIT:
        // They will be adjusted dynamically
        step_length = leg_reach * params.gait_factors.adaptive_length_factor;
        step_height = robot_height * params.gait_factors.adaptive_height_factor;
        break;
    }

    // Calculate dynamic limits based on robot dimensions
    float min_step_length = leg_reach * params.gait_factors.min_length_factor;
    float max_step_length = leg_reach * params.gait_factors.max_length_factor;
    float min_step_height = robot_height * params.gait_factors.min_height_factor;
    float max_step_height = robot_height * params.gait_factors.max_height_factor;

    // Verify that parameters are within dynamic limits
    step_length = constrainAngle(step_length, min_step_length, max_step_length);
    step_height = constrainAngle(step_height, min_step_height, max_step_height);
}

bool LocomotionSystem::checkJointLimits(int leg_index, const JointAngles &angles) {
    return model.checkJointLimits(leg_index, angles);
}

float LocomotionSystem::constrainAngle(float angle, float min_angle, float max_angle) {
    return model.constrainAngle(angle, min_angle, max_angle);
}

bool LocomotionSystem::validateParameters() {
    return model.validate();
}

void LocomotionSystem::adaptGaitToTerrain() {
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

bool LocomotionSystem::setLegPosition(int leg_index, const Point3D &position) {
    if (!system_enabled || leg_index < 0 || leg_index >= NUM_LEGS || !pose_ctrl)
        return false;
    if (!pose_ctrl->setLegPosition(leg_index, position, leg_positions, joint_angles)) {
        last_error = KINEMATICS_ERROR;
        return false;
    }
    return true;
}

bool LocomotionSystem::setStepParameters(float height, float length) {
    if (height < 15.0f || height > 50.0f || length < 20.0f || length > 80.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    step_height = height;
    step_length = length;
    return true;
}

bool LocomotionSystem::setParameters(const Parameters &new_params) {
    // Validar nuevos parameters
    if (new_params.hexagon_radius <= 0 || new_params.coxa_length <= 0 ||
        new_params.femur_length <= 0 || new_params.tibia_length <= 0) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params = new_params;
    return validateParameters();
}

bool LocomotionSystem::setControlFrequency(float frequency) {
    if (frequency < 10.0f || frequency > 200.0f) {
        last_error = PARAMETER_ERROR;
        return false;
    }

    params.control_frequency = frequency;
    return true;
}


float LocomotionSystem::calculateLegReach(int leg_index) {
    return params.coxa_length + params.femur_length + params.tibia_length;
}

void LocomotionSystem::adjustStepParameters() {
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

void LocomotionSystem::compensateForSlope() {
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

float LocomotionSystem::getStepLength() const {
    // Base step length según el tipo de marcha actual
    float base_step_length = step_length;

    // Factores de ajuste basados en parámetros del robot
    float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float max_safe_step = leg_reach * params.gait_factors.max_length_factor;

    // Ajuste por estabilidad - reducir paso si la estabilidad es baja
    float stability_factor = 1.0f;
    if (system_enabled) {
        float stability_index = const_cast<LocomotionSystem*>(this)->calculateStabilityIndex();
        if (stability_index < 0.5f) {
            stability_factor = 0.7f + 0.3f * stability_index; // Reducir hasta 70%
        }
    }

    // Ajuste por inclinación del terreno si hay IMU
    float terrain_factor = 1.0f;
    if (imu_interface && imu_interface->isConnected()) {
        IMUData imu_data = imu_interface->readIMU();
        if (imu_data.is_valid) {
            float total_tilt = sqrt(imu_data.roll * imu_data.roll + imu_data.pitch * imu_data.pitch);
            if (total_tilt > 10.0f) {
                terrain_factor = std::max(0.6f, 1.0f - (total_tilt - 10.0f) / 20.0f);
            }
        }
    }

    // Calcular longitud final del paso
    float calculated_step_length = base_step_length * stability_factor * terrain_factor;

    // Limitar dentro de rangos seguros basados en parámetros
    float min_safe_step = leg_reach * params.gait_factors.min_length_factor;
    calculated_step_length = std::max(min_safe_step, std::min(max_safe_step, calculated_step_length));

    return calculated_step_length;
}
