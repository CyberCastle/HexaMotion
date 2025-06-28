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

    // Initialize default DH parameters if custom parameters are not used
    if (!params.use_custom_dh_parameters) {
        // Per-leg base joint (coxa) theta offsets (degrees) for mounting orientation
        // Leg 0 points toward +X (0°), remaining legs are spaced 60° apart
        static const float base_theta_offsets[NUM_LEGS] = {
            0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f};
        for (int l = 0; l < NUM_LEGS; ++l) {
            // Simplified DH configuration to mimic angle_calculus.cpp behavior
            // Geometry: H_mm = coxa_length * sin(femur_angle) + femur_length * sin(tibia_angle) + tibia_length

            // Joint 1 (coxa): rotation about body Z-axis (horizontal plane)
            dh_transforms[l][0][0] = params.coxa_length;    // a0 - coxa length
            dh_transforms[l][0][1] = 0.0f;                  // alpha0 - no twist
            dh_transforms[l][0][2] = 0.0f;                  // d1 - no vertical offset
            dh_transforms[l][0][3] = base_theta_offsets[l]; // theta1 offset

            // Joint 2 (femur): vertical rotation to contribute to coxa_length * sin(femur_angle) height
            dh_transforms[l][1][0] = 0.0f;               // a1 - no horizontal extension
            dh_transforms[l][1][1] = 90.0f;              // alpha1 - twist to convert to Z movement
            dh_transforms[l][1][2] = params.coxa_length; // d2 - coxa offset in Z to allow sin(femur_angle)
            dh_transforms[l][1][3] = 0.0f;               // theta2 offset

            // Joint 3 (tibia): vertical rotation to contribute to femur_length * sin(tibia_angle) + tibia_length
            dh_transforms[l][2][0] = 0.0f;                                         // a2 - no horizontal extension
            dh_transforms[l][2][1] = 0.0f;                                         // alpha2 - no twist
            dh_transforms[l][2][2] = -(params.femur_length + params.tibia_length); // d3 - total vertical length
            dh_transforms[l][2][3] = 0.0f;                                         // theta3 offset
        }
    }
}

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) {
    // Inverse kinematics: Damped Least Squares solver using DH parameters

    // Transform target to leg coordinate system
    const float base_angle_deg = leg * LEG_ANGLE_SPACING;
    float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    local_target.x = p_target.x - base_x;
    local_target.y = p_target.y - base_y;
    local_target.z = p_target.z;

    // SIMPLIFIED: Basic workspace validation only (detailed validation done by WorkspaceValidator)
    float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float min_reach = std::abs(params.femur_length - params.tibia_length);
    float distance = sqrt(local_target.x * local_target.x +
                          local_target.y * local_target.y +
                          local_target.z * local_target.z);

    if (distance > max_reach * 0.98f || distance < min_reach * 1.02f) {
        // Target outside workspace - return safe default angles
        float coxa_angle = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
        coxa_angle = constrainAngle(coxa_angle, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        return JointAngles(coxa_angle, -45.0f, 60.0f);
    }

    // Initial guess based on target direction and realistic kinematics
    float coxa_start = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
    coxa_start = constrainAngle(coxa_start, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

    // Better initial guess for femur and tibia based on target position
    float target_distance_xy = sqrt(local_target.x * local_target.x + local_target.y * local_target.y);
    float target_height = -local_target.z; // Convert to positive height

    // Initial estimates for femur and tibia
    float femur_estimate = 0.0f;
    float tibia_estimate = 0.0f;

    // Optional: geometric approximation fallback
    if (femur_estimate == 0.0f && tibia_estimate == 0.0f && target_distance_xy > params.coxa_length && target_height > 0) {
        float remaining_xy = target_distance_xy - params.coxa_length;
        float leg_reach = sqrt(remaining_xy * remaining_xy + target_height * target_height);
        if (leg_reach <= (params.femur_length + params.tibia_length) * 0.95f) {
            float c = leg_reach;
            float a = params.femur_length;
            float b = params.tibia_length;
            float cos_alpha = (a * a + c * c - b * b) / (2 * a * c);
            if (cos_alpha >= -1.0f && cos_alpha <= 1.0f) {
                float alpha = acos(cos_alpha);
                float theta = atan2(target_height, remaining_xy);
                femur_estimate = (theta - alpha) * RADIANS_TO_DEGREES_FACTOR;
                float cos_beta = (a * a + b * b - c * c) / (2 * a * b);
                if (cos_beta >= -1.0f && cos_beta <= 1.0f) {
                    float beta = acos(cos_beta);
                    tibia_estimate = (M_PI - beta) * RADIANS_TO_DEGREES_FACTOR;
                }
            }
        }
    }

    // Clamp initial estimates to joint limits
    femur_estimate = constrainAngle(femur_estimate, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    tibia_estimate = constrainAngle(tibia_estimate, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    JointAngles current_angles(coxa_start, femur_estimate, tibia_estimate);

    // Clamp to joint limits
    current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
    current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    // DLS iterative solution - production-ready 3x3 solver
    const int max_iterations = params.ik.max_iterations;
    const float tolerance = 0.001f;
    const float dls_coefficient = 0.05f;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Calculate current position using forward kinematics
        Point3D current_pos = forwardKinematics(leg, current_angles);

        // Position error (3D)
        Eigen::Vector3f position_error3;
        position_error3 << (local_target.x - current_pos.x),
            (local_target.y - current_pos.y),
            (local_target.z - current_pos.z);

        // Check for convergence
        if (position_error3.norm() < tolerance)
            break;

        // 3x3 Jacobian for position only
        Eigen::Matrix3f jacobian3 = calculateJacobian(leg, current_angles, p_target);

        // Damped least squares inverse: J_inv = J^T * (J*J^T + λ^2 I)^(-1)
        Eigen::Matrix3f JJT3 = jacobian3 * jacobian3.transpose();
        Eigen::Matrix3f identity3 = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f damped_inv3 = (JJT3 + dls_coefficient * dls_coefficient * identity3).inverse();
        Eigen::Matrix3f jacobian_inverse3 = jacobian3.transpose() * damped_inv3;

        // Calculate joint angle changes
        Eigen::Vector3f angle_delta = jacobian_inverse3 * position_error3;

        // Apply adaptive step scaling to prevent large jumps
        float step_scale = 1.0f;
        float max_angle_change = 5.0f; // Max 5 degrees per iteration (more conservative)
        float max_delta = std::max({std::abs(angle_delta(0)), std::abs(angle_delta(1)), std::abs(angle_delta(2))});
        if (max_delta > math_utils::degreesToRadians(max_angle_change)) {
            step_scale = math_utils::degreesToRadians(max_angle_change) / max_delta;
        }

        // Update joint angles (convert from radians to degrees)
        current_angles.coxa += angle_delta(0) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.femur += angle_delta(1) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.tibia += angle_delta(2) * RADIANS_TO_DEGREES_FACTOR * step_scale;

        // Normalize angles
        current_angles.coxa = normalizeAngle(current_angles.coxa);
        current_angles.femur = normalizeAngle(current_angles.femur);
        current_angles.tibia = normalizeAngle(current_angles.tibia);

        // Apply joint limits
        current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }

    return current_angles;
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) const {
    // Forward kinematics: full DH transform chain
    Eigen::Matrix4f transform = legTransform(leg_index, angles);
    return Point3D{transform(0, 3), transform(1, 3), transform(2, 3)};
}

Eigen::Matrix4f RobotModel::legTransform(int leg_index, const JointAngles &q) const {
    const float base_angle_deg = leg_index * LEG_ANGLE_SPACING;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T(0, 3) = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    T(1, 3) = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));
    T.block<3, 3>(0, 0) = math_utils::rotationMatrixZ(base_angle_deg);

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 0; j < DOF_PER_LEG; ++j) {
        // Extract DH parameters for this joint
        float a = dh_transforms[leg_index][j][0];      // link length
        float alpha = dh_transforms[leg_index][j][1];  // twist angle
        float d = dh_transforms[leg_index][j][2];      // link offset
        float theta0 = dh_transforms[leg_index][j][3]; // joint angle offset
        float theta = theta0 + joint_deg[j];           // total joint angle
        T *= math_utils::dhTransform(a, alpha, d, theta);
    }

    return T;
}

Eigen::Matrix3f RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const {
    // Calculate Jacobian from DH matrices along kinematic chain
    // Based on syropod_highlevel_controller implementation

    // Get base transform for this leg
    const float base_angle_deg = leg * LEG_ANGLE_SPACING;
    Eigen::Matrix4f T_base = Eigen::Matrix4f::Identity();
    T_base(0, 3) = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    T_base(1, 3) = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));
    T_base.block<3, 3>(0, 0) = math_utils::rotationMatrixZ(base_angle_deg);

    // Calculate transforms and positions for each joint
    std::vector<Eigen::Matrix4f> transforms(DOF_PER_LEG + 1);
    transforms[0] = T_base;

    const float joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Build transforms step by step using DH parameters
    for (int j = 0; j < DOF_PER_LEG; ++j) {
        // Extract DH parameters for this joint
        float a = dh_transforms[leg][j][0];      // link length
        float alpha = dh_transforms[leg][j][1];  // twist angle
        float d = dh_transforms[leg][j][2];      // link offset
        float theta0 = dh_transforms[leg][j][3]; // joint angle offset
        float theta = theta0 + joint_deg[j];     // total joint angle

        transforms[j + 1] = transforms[j] * math_utils::dhTransform(a, alpha, d, theta);
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
