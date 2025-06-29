#include "HexaModel.h"
#include "hexamotion_constants.h"

/**
 * @file robot_model.cpp
 * @brief Implementation of the kinematic robot model.
 */
#include <limits>
#include <math.h>
#include <vector>

RobotModel::RobotModel(const Parameters &p) : params(p) {
    initializeDH();
}

void RobotModel::initializeDH() {

    // Use default DH parameters if no custom parameters provided
    if (!params.use_custom_dh_parameters) {
        // Per-leg base orientations around the hexagonal body (degrees)
        static const float base_theta_offsets[NUM_LEGS] = {
            0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};
        for (int l = 0; l < NUM_LEGS; ++l) {
            // Base transform from body center to leg mount
            dh_transforms[l][0][0] = params.hexagon_radius; // a
            dh_transforms[l][0][1] = 0.0f;                 // alpha
            dh_transforms[l][0][2] = 0.0f;                 // d
            dh_transforms[l][0][3] = base_theta_offsets[l]; // theta

            // Coxa link
            dh_transforms[l][1][0] = params.coxa_length; // a
            dh_transforms[l][1][1] = 90.0f;              // alpha
            dh_transforms[l][1][2] = 0.0f;               // d
            dh_transforms[l][1][3] = 0.0f;               // theta offset

            // Femur link
            dh_transforms[l][2][0] = params.femur_length; // a
            dh_transforms[l][2][1] = 0.0f;               // alpha
            dh_transforms[l][2][2] = 0.0f;               // d
            dh_transforms[l][2][3] = 0.0f;               // theta offset

            // Tibia link
            dh_transforms[l][3][0] = params.tibia_length; // a
            dh_transforms[l][3][1] = 0.0f;               // alpha
            dh_transforms[l][3][2] = 0.0f;               // d
            dh_transforms[l][3][3] = -90.0f;             // theta offset
        }
    }
}

// Damped Least Squares (DLS) iterative inverse kinematics
// Based on CSIRO syropod_highlevel_controller implementation
JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) {

    // Transform target to leg coordinate system
    Eigen::Matrix4f T_base = math_utils::dhTransform(
        dh_transforms[leg][0][0],
        dh_transforms[leg][0][1],
        dh_transforms[leg][0][2],
        dh_transforms[leg][0][3]);
    Eigen::Vector4f target_vec(p_target.x, p_target.y, p_target.z, 1.0f);
    Eigen::Vector4f local_vec = T_base.inverse() * target_vec;

    Point3D local_target(local_vec(0), local_vec(1), local_vec(2));

    // Quick workspace check using centralized reachability function
    float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float min_reach = std::abs(params.femur_length - params.tibia_length);

    // Check workspace bounds using centralized function (allow margin for numerical precision)
    if (!math_utils::isPointReachable(local_target, min_reach * IK_MIN_REACH_MARGIN, max_reach * IK_MAX_REACH_MARGIN)) {
        float coxa_angle = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
        coxa_angle = constrainAngle(coxa_angle, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

        // Determine if target is too far or too close
        float target_distance = math_utils::magnitude(local_target);
        if (target_distance > max_reach * IK_MAX_REACH_MARGIN) {
            // Target too far - return extended pose within joint limits
            return JointAngles(coxa_angle, IK_PRIMARY_FEMUR_ANGLE, IK_PRIMARY_TIBIA_ANGLE);
        } else {
            // Target too close - return retracted pose within joint limits
            return JointAngles(coxa_angle, IK_HIGH_FEMUR_ANGLE, IK_HIGH_TIBIA_ANGLE);
        }
    }

    float coxa_start = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
    coxa_start = constrainAngle(coxa_start, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
    JointAngles current_angles(coxa_start, IK_PRIMARY_FEMUR_ANGLE, IK_PRIMARY_TIBIA_ANGLE);

    for (int iter = 0; iter < IK_MAX_ITERATIONS; ++iter) {
        Point3D current_pos = forwardKinematics(leg, current_angles);

        Eigen::Vector3f position_delta;
        position_delta << (local_target.x - current_pos.x),
            (local_target.y - current_pos.y),
            (local_target.z - current_pos.z);

        if (position_delta.norm() < IK_TOLERANCE * POSITION_TOLERANCE) {
            break;
        }

        Eigen::Matrix3f jacobian = calculateJacobian(leg, current_angles, p_target);
        Eigen::Matrix3f JJT = jacobian * jacobian.transpose();
        Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f damped_inv = (JJT + IK_DLS_COEFFICIENT * IK_DLS_COEFFICIENT * identity).inverse();
        Eigen::Matrix3f jacobian_inverse = jacobian.transpose() * damped_inv;

        Eigen::Vector3f angle_delta = jacobian_inverse * position_delta;

        current_angles.coxa += angle_delta(0) * RADIANS_TO_DEGREES_FACTOR;
        current_angles.femur += angle_delta(1) * RADIANS_TO_DEGREES_FACTOR;
        current_angles.tibia += angle_delta(2) * RADIANS_TO_DEGREES_FACTOR;

        current_angles.coxa = normalizeAngle(current_angles.coxa);
        current_angles.femur = normalizeAngle(current_angles.femur);
        current_angles.tibia = normalizeAngle(current_angles.tibia);

        if (params.ik.clamp_joints) {
            current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
            current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
            current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
        }
    }

    return current_angles;
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) const {
    Eigen::Matrix4f transform = legTransform(leg_index, angles);
    Point3D position;
    position.x = transform(0, 3);
    position.y = transform(1, 3);
    position.z = transform(2, 3);
    return position;
}

Eigen::Matrix4f RobotModel::legTransform(int leg_index, const JointAngles &q) const {
    Eigen::Matrix4f T = math_utils::dhTransform(
        dh_transforms[leg_index][0][0],
        dh_transforms[leg_index][0][1],
        dh_transforms[leg_index][0][2],
        dh_transforms[leg_index][0][3]);

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        // Extract DH parameters for this joint
        float a = dh_transforms[leg_index][j][0];      // link length
        float alpha = dh_transforms[leg_index][j][1];  // twist angle
        float d = dh_transforms[leg_index][j][2];      // link offset
        float theta0 = dh_transforms[leg_index][j][3]; // joint angle offset
        float theta = theta0 + joint_deg[j - 1];       // total joint angle
        T *= math_utils::dhTransform(a, alpha, d, theta);
    }

    return T;
}

Eigen::Matrix3f RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const {
    // Calculate Jacobian from DH matrices along kinematic chain
    // Based on syropod_highlevel_controller implementation

    // Calculate transforms and positions for each joint
    std::vector<Eigen::Matrix4f> transforms(DOF_PER_LEG + 1);

    // Base transform
    transforms[0] = math_utils::dhTransform(
        dh_transforms[leg][0][0],
        dh_transforms[leg][0][1],
        dh_transforms[leg][0][2],
        dh_transforms[leg][0][3]);

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Build transforms step by step using DH parameters
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        float a = dh_transforms[leg][j][0];
        float alpha = dh_transforms[leg][j][1];
        float d = dh_transforms[leg][j][2];
        float theta0 = dh_transforms[leg][j][3];
        float theta = theta0 + joint_deg[j - 1];

        transforms[j] = transforms[j - 1] * math_utils::dhTransform(a, alpha, d, theta);
    }

    // End-effector transform
    Eigen::Matrix4f T_final = transforms[DOF_PER_LEG];

    // End-effector position
    Eigen::Vector3f pe = T_final.block<3, 1>(0, 3);

    // Initialize Jacobian matrix (3x3 for position only)
    Eigen::Matrix3f jacobian;

    // First joint (base joint) - use standard z-axis
    Eigen::Vector3f z0(0, 0, 1);
    Eigen::Vector3f p0 = transforms[0].block<3, 1>(0, 3);
    jacobian.col(0) = z0.cross(pe - p0);

    // Remaining joints
    for (int j = 1; j < DOF_PER_LEG; ++j) {
        Eigen::Vector3f zj = transforms[j].block<3, 1>(0, 2); // z-axis of joint j
        Eigen::Vector3f pj = transforms[j].block<3, 1>(0, 3); // position of joint j
        jacobian.col(j) = zj.cross(pe - pj);
    }

    return jacobian;
}

Eigen::Matrix3f RobotModel::analyticJacobian(int leg, const JointAngles &q) const {
    // Legacy method - kept for compatibility
    Point3D dummy_target = forwardKinematics(leg, q);
    return calculateJacobian(leg, q, dummy_target);
}

bool RobotModel::checkJointLimits(int leg_index, const JointAngles &angles) const {
    return (angles.coxa >= params.coxa_angle_limits[0] && angles.coxa <= params.coxa_angle_limits[1] &&
            angles.femur >= params.femur_angle_limits[0] && angles.femur <= params.femur_angle_limits[1] &&
            angles.tibia >= params.tibia_angle_limits[0] && angles.tibia <= params.tibia_angle_limits[1]);
}

float RobotModel::constrainAngle(float angle, float min_angle, float max_angle) const {
    // First normalize angle to [-180, 180] range to handle wraparound
    float normalized_angle = normalizeAngle(angle);

    // Then clamp to the specified joint limits
    return std::max(min_angle, std::min(max_angle, normalized_angle));
}

float RobotModel::normalizeAngle(float angle_deg) const {
    // Normalize angle to [-180, 180] range following syropod implementation
    // This handles angle wraparound issues that can occur during IK iteration

    // Convert to [-PI, PI] range first
    float angle_rad = angle_deg * M_PI / 180.0f;

    // Normalize to [-PI, PI] using atan2 trick
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));

    // Convert back to degrees
    return angle_rad * RADIANS_TO_DEGREES_FACTOR;
}

bool RobotModel::validate() const {
    return (params.hexagon_radius > 0 && params.coxa_length > 0 && params.femur_length > 0 && params.tibia_length > 0 &&
            params.robot_height > 0 && params.control_frequency > 0);
}

std::pair<float, float> RobotModel::calculateHeightRange() const {
    float min_h = std::numeric_limits<float>::max();
    float max_h = -std::numeric_limits<float>::max();

    // Workspace analysis: discretize the joint configuration space
    // Based on "Introduction to Robotics" - Craig and "Robotics: Modelling, Planning and Control" - Siciliano
    const int resolution = WORKSPACE_RESOLUTION; // discretization resolution

    const float coxa_step = (params.coxa_angle_limits[1] - params.coxa_angle_limits[0]) / resolution;
    const float femur_step = (params.femur_angle_limits[1] - params.femur_angle_limits[0]) / resolution;
    const float tibia_step = (params.tibia_angle_limits[1] - params.tibia_angle_limits[0]) / resolution;

    // Evaluate the entire workspace of valid joint configurations
    for (int i = 0; i <= resolution; i++) {
        for (int j = 0; j <= resolution; j++) {
            for (int k = 0; k <= resolution; k++) {
                float coxa = params.coxa_angle_limits[0] + i * coxa_step;
                float femur = params.femur_angle_limits[0] + j * femur_step;
                float tibia = params.tibia_angle_limits[0] + k * tibia_step;
                JointAngles q(coxa, femur, tibia);

                // Check that angles are within limits
                if (!checkJointLimits(0, q))
                    continue;

                Point3D pos = forwardKinematics(0, q);

                // Calculate body height considering the robot's physical offset
                // pos.z is negative when the leg is below the body
                float height = -pos.z + params.height_offset;

                // Only consider physically valid heights (positive)
                if (height > 0) {
                    min_h = std::min(min_h, height);
                    max_h = std::max(max_h, height);
                }
            }
        }
    }

    // If no valid configurations were found, indicates parameter error
    if (min_h == std::numeric_limits<float>::max()) {
        // Return values indicating error - inconsistent robot parameters
        return {-1.0f, -1.0f};
    }

    return {min_h, max_h};
}

Point3D RobotModel::getLegOrigin(int leg) const {
    if (leg < 0 || leg >= NUM_LEGS) {
        return Point3D(0, 0, 0);
    }

    // Calculate leg origin based on hexagon geometry
    float angle = leg * M_PI / 3.0f; // 60 degrees between legs
    float x = params.hexagon_radius * cos(angle);
    float y = params.hexagon_radius * sin(angle);
    float z = 0.0f; // At body level

    return Point3D(x, y, z);
}
