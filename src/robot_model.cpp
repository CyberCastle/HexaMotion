#include "HexaModel.h"
#include "hexamotion_constants.h"

/**
 * @file robot_model.cpp
 * @brief Implementation of the kinematic robot model.
 */
#include <limits>
#include <math.h>
#include <vector>

// Per-leg base orientation offsets (degrees)
static const double BASE_THETA_OFFSETS[NUM_LEGS] = {0.0f, -60.0f, -120.0f, 180.0f, 120.0f, 60.0f};

RobotModel::RobotModel(const Parameters &p) : params(p) {
    initializeDH();
}

void RobotModel::initializeDH() {

    // Initialize default DH parameters if custom parameters are not used
    if (!params.use_custom_dh_parameters) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            // ── Fila 0: base rígida ───────────────────────────
            dh_transforms[l][0][0] = params.hexagon_radius; // a0 = 200mm
            dh_transforms[l][0][1] = 0.0f;                  // alpha0
            dh_transforms[l][0][2] = 0.0f;                  // d1
            dh_transforms[l][0][3] = BASE_THETA_OFFSETS[l]; // θ0  (fijo)

            // ── Fila 1: servo yaw ─────────────────────────────
            dh_transforms[l][1][0] = 0.0f;  // a1
            dh_transforms[l][1][1] = 90.0f; // alpha1 (+90°)
            dh_transforms[l][1][2] = 0.0f;  // d2
            dh_transforms[l][1][3] = 0.0f;  // θ1 offset (suma ψ)

            // ── Fila 2: servo hip-pitch + coxa ───────────────
            dh_transforms[l][2][0] = params.coxa_length; // a2 = 50
            dh_transforms[l][2][1] = 90.0f;              // alpha2 (+90°)
            dh_transforms[l][2][2] = 0.0f;               // d3
            dh_transforms[l][2][3] = 0.0f;               // θ2 offset (suma θ₁)

            // ── Fila 3: servo knee-pitch + tibia ─────────────
            dh_transforms[l][3][0] = params.femur_length; // a3 = 101
            dh_transforms[l][3][1] = 0.0f;                // alpha3
            dh_transforms[l][3][2] = params.tibia_length; // d4 = 208
            dh_transforms[l][3][3] = 0.0f;                // θ3 offset (suma θ₂)
        }
    } else {
        // Copy custom DH parameters provided in params.dh_parameters
        for (int l = 0; l < NUM_LEGS; ++l) {
            for (int j = 0; j < DOF_PER_LEG + 1; ++j) {
                for (int k = 0; k < 4; ++k) {
                    dh_transforms[l][j][k] = params.dh_parameters[l][j][k];
                }
            }
        }
    }
}

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::solveIK(int leg, const Point3D &local_target, JointAngles current_angles) const {
    const double tolerance = 0.001f;
    const double dls_coefficient = 0.05f;

    for (int iter = 0; iter < params.ik.max_iterations; ++iter) {
        const double joint_deg[DOF_PER_LEG] = {current_angles.coxa, current_angles.femur, current_angles.tibia};
        Eigen::Matrix4d current_tf = Eigen::Matrix4d::Identity();
        for (int j = 1; j <= DOF_PER_LEG; ++j) {
            double a = dh_transforms[leg][j][0];
            double alpha = dh_transforms[leg][j][1];
            double d = dh_transforms[leg][j][2];
            double theta_off = dh_transforms[leg][j][3];
            double theta = theta_off + joint_deg[j - 1];
            double alpha_rad = math_utils::degreesToRadians(alpha);
            double theta_rad = math_utils::degreesToRadians(theta);
            current_tf *= math_utils::dhTransform(a, alpha_rad, d, theta_rad);
        }
        Point3D current_pos{current_tf(0, 3), current_tf(1, 3), current_tf(2, 3)};

        Eigen::Vector3d position_error3;
        position_error3 << (local_target.x - current_pos.x),
            (local_target.y - current_pos.y),
            (local_target.z - current_pos.z);

        if (position_error3.norm() < tolerance)
            break;

        std::vector<Eigen::Matrix4d> transforms(DOF_PER_LEG + 1);
        transforms[0] = Eigen::Matrix4d::Identity();
        for (int j = 1; j <= DOF_PER_LEG; ++j) {
            double a = dh_transforms[leg][j][0];
            double alpha = dh_transforms[leg][j][1];
            double d = dh_transforms[leg][j][2];
            double theta_off = dh_transforms[leg][j][3];
            double theta = theta_off + joint_deg[j - 1];
            double alpha_rad = math_utils::degreesToRadians(alpha);
            double theta_rad = math_utils::degreesToRadians(theta);
            transforms[j] = transforms[j - 1] * math_utils::dhTransform(a, alpha_rad, d, theta_rad);
        }

        Eigen::Matrix3d jacobian_pos;
        Eigen::Vector3d pe = transforms[DOF_PER_LEG].block<3, 1>(0, 3);
        Eigen::Vector3d z0(0, 0, 1);
        Eigen::Vector3d p0 = transforms[0].block<3, 1>(0, 3);
        jacobian_pos.col(0) = z0.cross(pe - p0);
        for (int j = 1; j < DOF_PER_LEG; ++j) {
            Eigen::Vector3d zj = transforms[j].block<3, 1>(0, 2);
            Eigen::Vector3d pj = transforms[j].block<3, 1>(0, 3);
            jacobian_pos.col(j) = zj.cross(pe - pj);
        }

        Eigen::Matrix3d JJT3 = jacobian_pos * jacobian_pos.transpose();
        Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d damped_inv3 = (JJT3 + dls_coefficient * dls_coefficient * identity3).inverse();
        Eigen::Matrix3d jacobian_inverse3 = jacobian_pos.transpose() * damped_inv3;

        Eigen::Vector3d angle_delta = jacobian_inverse3 * position_error3;

        double step_scale = 1.0f;
        double max_angle_change = 5.0f;
        double max_delta = std::max({std::abs(angle_delta(0)), std::abs(angle_delta(1)), std::abs(angle_delta(2))});
        if (max_delta > math_utils::degreesToRadians(max_angle_change)) {
            step_scale = math_utils::degreesToRadians(max_angle_change) / max_delta;
        }

        current_angles.coxa += angle_delta(0) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.femur += angle_delta(1) * RADIANS_TO_DEGREES_FACTOR * step_scale;
        current_angles.tibia += angle_delta(2) * RADIANS_TO_DEGREES_FACTOR * step_scale;

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

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) const {
    // Inverse kinematics: Damped Least Squares solver using DH parameters

    // Transform target to leg coordinate system
    const double base_angle_deg = BASE_THETA_OFFSETS[leg];
    double base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    double base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    double dx = p_target.x - base_x;
    double dy = p_target.y - base_y;
    double angle_rad = math_utils::degreesToRadians(-base_angle_deg);
    local_target.x = cos(angle_rad) * dx - sin(angle_rad) * dy;
    local_target.y = sin(angle_rad) * dx + cos(angle_rad) * dy;
    local_target.z = p_target.z;

    // Basic workspace validation only (detailed validation done by WorkspaceValidator)
    double max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    double min_reach = std::abs(params.femur_length - params.tibia_length);
    double distance = sqrt(local_target.x * local_target.x +
                           local_target.y * local_target.y +
                           local_target.z * local_target.z);

    if (distance > max_reach * 0.98f || distance < min_reach * 1.02f) {
        // Target outside workspace - return safe default angles
        double coxa_angle = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
        coxa_angle = constrainAngle(coxa_angle, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        return JointAngles(coxa_angle, -45.0f, 60.0f);
    }

    // Initial guess based on target direction and realistic kinematics
    double coxa_start = atan2(local_target.y, local_target.x) * RADIANS_TO_DEGREES_FACTOR;
    coxa_start = constrainAngle(coxa_start, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

    // Initial estimates for femur and tibia based only on DH model
    double femur_estimate = 0.0f;
    double tibia_estimate = 0.0f;

    // Clamp initial estimates to joint limits
    femur_estimate = constrainAngle(femur_estimate, params.femur_angle_limits[0], params.femur_angle_limits[1]);
    tibia_estimate = constrainAngle(tibia_estimate, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

    JointAngles current_angles(coxa_start, femur_estimate, tibia_estimate);

    // Clamp to joint limits

    if (params.ik.clamp_joints) {
        current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }

    return solveIK(leg, local_target, current_angles);
}

JointAngles RobotModel::inverseKinematicsCurrent(int leg, const JointAngles &current_angles,
                                                 const Point3D &p_target) const {
    const double base_angle_deg = BASE_THETA_OFFSETS[leg];
    double base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    double base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    double dx = p_target.x - base_x;
    double dy = p_target.y - base_y;
    double angle_rad = math_utils::degreesToRadians(-base_angle_deg);
    local_target.x = cos(angle_rad) * dx - sin(angle_rad) * dy;
    local_target.y = sin(angle_rad) * dx + cos(angle_rad) * dy;
    local_target.z = p_target.z;

    JointAngles start = current_angles;
    if (params.ik.clamp_joints) {
        start.coxa = constrainAngle(start.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        start.femur = constrainAngle(start.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        start.tibia = constrainAngle(start.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
    }

    return solveIK(leg, local_target, start);
}

Point3D RobotModel::forwardKinematics(int leg_index, const JointAngles &angles) const {
    // Forward kinematics: compute full DH transform chain
    // Use double precision for improved stability
    Eigen::Matrix4d transform = legTransform(leg_index, angles);
    return Point3D{transform(0, 3), transform(1, 3), transform(2, 3)};
}

Point3D RobotModel::getAnalyticLegBasePosition(int leg_index) const {
    // Compute base position using nominal leg offset angle
    const double angle_deg = BASE_THETA_OFFSETS[leg_index];
    const double angle_rad = math_utils::degreesToRadians(angle_deg);

    double x = params.hexagon_radius * cos(angle_rad);
    double y = params.hexagon_radius * sin(angle_rad);

    return Point3D{x, y, 0.0f};
}

Point3D RobotModel::getDHLegBasePosition(int leg_index) const {
    // Get only the base transform (without joint angles)
    Eigen::Matrix4d base_transform = math_utils::dhTransform(
        dh_transforms[leg_index][0][0],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][1]),
        dh_transforms[leg_index][0][2],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][3]));

    return Point3D{base_transform(0, 3), base_transform(1, 3), base_transform(2, 3)};
}

Eigen::Matrix4d RobotModel::legTransform(int leg_index, const JointAngles &q) const {
    // Base transform from DH parameters (body center to leg mount)
    Eigen::Matrix4d T = math_utils::dhTransform<double>(
        dh_transforms[leg_index][0][0],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][1]),
        dh_transforms[leg_index][0][2],
        math_utils::degreesToRadians(dh_transforms[leg_index][0][3]));

    const double joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        // Extract DH parameters for this joint
        double a = dh_transforms[leg_index][j][0];                                   // link length
        double alpha = math_utils::degreesToRadians(dh_transforms[leg_index][j][1]); // twist angle
        double d = dh_transforms[leg_index][j][2];                                   // link offset
        double theta0 = dh_transforms[leg_index][j][3];                              // joint offset
        double theta = theta0 + joint_deg[j - 1];                                    // total joint angle
        T *= math_utils::dhTransform<double>(a, alpha, d, math_utils::degreesToRadians(theta));
    }

    return T;
}

Eigen::Matrix3d RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &) const {
    // Calculate Jacobian from DH matrices along the kinematic chain
    // Based on syropod_highlevel_controller implementation
    // Base transform for this leg
    Eigen::Matrix4d T_base = math_utils::dhTransform<double>(
        dh_transforms[leg][0][0],
        math_utils::degreesToRadians(dh_transforms[leg][0][1]),
        dh_transforms[leg][0][2],
        math_utils::degreesToRadians(dh_transforms[leg][0][3]));

    std::vector<Eigen::Matrix4d> transforms(DOF_PER_LEG + 1);
    transforms[0] = T_base;

    const double joint_deg[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Build transforms step by step using DH parameters
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        // Extract DH parameters for this joint
        double a = dh_transforms[leg][j][0];                                   // link length
        double alpha = math_utils::degreesToRadians(dh_transforms[leg][j][1]); // twist angle
        double d = dh_transforms[leg][j][2];                                   // link offset
        double theta0 = dh_transforms[leg][j][3];                              // joint offset
        double theta = theta0 + joint_deg[j - 1];                              // total joint angle
        transforms[j] = transforms[j - 1] *
                        math_utils::dhTransform<double>(a, alpha, d, math_utils::degreesToRadians(theta));
    }

    // End-effector transform
    Eigen::Matrix4d T_final = transforms[DOF_PER_LEG];

    // End-effector position
    Eigen::Vector3d pe = T_final.block<3, 1>(0, 3);

    // Initialize Jacobian matrix (3x3 for position only)
    Eigen::Matrix3d jacobian;

    // First joint (base joint) - use standard z-axis
    Eigen::Vector3d z0(0.0, 0.0, 1.0);
    Eigen::Vector3d p0 = transforms[0].block<3, 1>(0, 3);
    jacobian.col(0) = (z0.cross(pe - p0)).cast<double>();

    // Remaining joints
    for (int j = 1; j < DOF_PER_LEG; ++j) {
        Eigen::Vector3d zj = transforms[j].block<3, 1>(0, 2); // z-axis of joint j
        Eigen::Vector3d pj = transforms[j].block<3, 1>(0, 3); // position of joint j
        jacobian.col(j) = (zj.cross(pe - pj)).cast<double>();
    }

    return jacobian;
}

bool RobotModel::checkJointLimits(int leg_index, const JointAngles &angles) const {
    return (angles.coxa >= params.coxa_angle_limits[0] && angles.coxa <= params.coxa_angle_limits[1] &&
            angles.femur >= params.femur_angle_limits[0] && angles.femur <= params.femur_angle_limits[1] &&
            angles.tibia >= params.tibia_angle_limits[0] && angles.tibia <= params.tibia_angle_limits[1]);
}

double RobotModel::constrainAngle(double angle, double min_angle, double max_angle) const {
    // First normalize angle to [-180, 180] range to handle wraparound
    double normalized_angle = normalizeAngle(angle);

    // Then clamp to the specified joint limits
    return std::max(min_angle, std::min(max_angle, normalized_angle));
}

double RobotModel::normalizeAngle(double angle_deg) const {
    // Normalize angle to [-180, 180] range following syropod implementation
    // This handles angle wraparound issues that can occur during IK iteration

    // Convert to [-PI, PI] range first
    double angle_rad = angle_deg * M_PI / 180.0f;

    // Normalize to [-PI, PI] using atan2 trick
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));

    // Convert back to degrees
    return angle_rad * RADIANS_TO_DEGREES_FACTOR;
}

bool RobotModel::validate() const {
    return (params.hexagon_radius > 0 && params.coxa_length > 0 && params.femur_length > 0 && params.tibia_length > 0 &&
            params.robot_height > 0 && params.control_frequency > 0);
}

std::pair<double, double> RobotModel::calculateHeightRange() const {
    double min_h = std::numeric_limits<double>::max();
    double max_h = -std::numeric_limits<double>::max();

    // Workspace analysis: discretize the joint configuration space
    // Based on "Introduction to Robotics" - Craig and "Robotics: Modelling, Planning and Control" - Siciliano
    const int resolution = WORKSPACE_RESOLUTION; // discretization resolution

    const double coxa_step = (params.coxa_angle_limits[1] - params.coxa_angle_limits[0]) / resolution;
    const double femur_step = (params.femur_angle_limits[1] - params.femur_angle_limits[0]) / resolution;
    const double tibia_step = (params.tibia_angle_limits[1] - params.tibia_angle_limits[0]) / resolution;

    // Evaluate the entire workspace of valid joint configurations
    for (int i = 0; i <= resolution; i++) {
        for (int j = 0; j <= resolution; j++) {
            for (int k = 0; k <= resolution; k++) {
                double coxa = params.coxa_angle_limits[0] + i * coxa_step;
                double femur = params.femur_angle_limits[0] + j * femur_step;
                double tibia = params.tibia_angle_limits[0] + k * tibia_step;
                JointAngles q(coxa, femur, tibia);

                // Check that angles are within limits
                if (!checkJointLimits(0, q))
                    continue;

                Point3D pos = forwardKinematics(0, q);

                // Calculate body height considering the robot's physical offset
                // pos.z is negative when the leg is below the body
                double height = -pos.z + params.height_offset;

                // Only consider physically valid heights (positive)
                if (height > 0) {
                    min_h = std::min(min_h, height);
                    max_h = std::max(max_h, height);
                }
            }
        }
    }

    // If no valid configurations were found, indicates parameter error
    if (min_h == std::numeric_limits<double>::max()) {
        // Return values indicating error - inconsistent robot parameters
        return {-1.0f, -1.0f};
    }

    return {min_h, max_h};
}

Pose RobotModel::getPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &leg_frame_pose) const {
    // Get the full transform from robot frame to leg frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    // Transform the leg frame pose to robot frame
    return leg_frame_pose.transform(transform);
}

Pose RobotModel::getPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose) const {
    // Get the full transform from robot frame to leg frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    // Transform the robot frame pose to leg frame (inverse transform)
    return robot_frame_pose.transform(transform.inverse());
}

Pose RobotModel::getTipPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &tip_frame_pose) const {
    // Get the full transform from robot frame to tip frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    // Transform the tip frame pose to robot frame
    return tip_frame_pose.transform(transform);
}

Pose RobotModel::getTipPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose) const {
    // Get the full transform from robot frame to tip frame
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    // Transform the robot frame pose to tip frame (inverse transform)
    return robot_frame_pose.transform(transform.inverse());
}
