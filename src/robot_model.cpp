#include "robot_model.h"
#include "hexamotion_constants.h"

/**
 * @file robot_model.cpp
 * @brief Implementation of the kinematic robot model.
 */
#include <limits>
#include <math.h>
#include <vector>

// Eliminar la definición de BASE_THETA_OFFSETS aquí, y moverla a hexamotion_constants.h

RobotModel::RobotModel(const Parameters &p) : params(p) {
    // Convert configuration angles from degrees to radians for internal use
    // Keep original parameters in degrees for configuration
    for (int i = 0; i < 2; ++i) {
        coxa_angle_limits_rad[i] = math_utils::degreesToRadians(params.coxa_angle_limits[i]);
        femur_angle_limits_rad[i] = math_utils::degreesToRadians(params.femur_angle_limits[i]);
        tibia_angle_limits_rad[i] = math_utils::degreesToRadians(params.tibia_angle_limits[i]);
    }

    max_angular_velocity_rad = math_utils::degreesToRadians(params.max_angular_velocity);
    body_comp_max_tilt_rad = math_utils::degreesToRadians(params.body_comp.max_tilt_deg);

    initializeDH();
}

void RobotModel::initializeDH() {

    // Initialize default DH parameters if custom parameters are not used.
    // The analytic model pitches the femur and tibia about the Y axis. We
    // replicate that behavior by later applying dhTransformY on rows 2 and 3.
    if (!params.use_custom_dh_parameters) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            // ── Row 0: fixed base ────────────────────────────
            dh_transforms[l][0][0] = params.hexagon_radius; // a0 = 200mm
            dh_transforms[l][0][1] = 0.0f;                  // alpha0
            dh_transforms[l][0][2] = 0.0f;                  // d1
            dh_transforms[l][0][3] = BASE_THETA_OFFSETS[l]; // θ0 (fixed)

            // ── Row 1: yaw servo + coxa ─────────────────────
            dh_transforms[l][1][0] = params.coxa_length; // a1 = 50
            dh_transforms[l][1][1] = 0.0f;               // alpha1
            dh_transforms[l][1][2] = 0.0f;               // d2
            dh_transforms[l][1][3] = 0.0f;               // θ1 offset (adds ψ)

            // ── Row 2: femur pitch servo ────────────────────
            dh_transforms[l][2][0] = params.femur_length; // a2 = 101
            dh_transforms[l][2][1] = 0.0f;                // alpha2
            dh_transforms[l][2][2] = 0.0f;                // d3
            dh_transforms[l][2][3] = 0.0f;                // θ2 offset (adds θ₁)

            // ── Row 3: knee pitch servo + tibia ─────────────
            dh_transforms[l][3][0] = 0.0f;                 // a3
            dh_transforms[l][3][1] = 0.0f;                 // alpha3
            dh_transforms[l][3][2] = -params.tibia_length; // d4 = -208
            dh_transforms[l][3][3] = 0.0f;                 // θ3 offset (adds θ₂)
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
    const double tolerance = IK_TOLERANCE;
    const double dls_coefficient = IK_DLS_COEFFICIENT;
    const double max_angle_change = IK_MAX_ANGLE_STEP * DEGREES_TO_RADIANS_FACTOR; // Max angle change per iteration

    for (int iter = 0; iter < params.ik.max_iterations; ++iter) {
        // Calculate current position in local leg frame using FK
        Point3D current_pos_global = forwardKinematicsGlobalCoordinates(leg, current_angles);
        Point3D current_pos = transformGlobalToLocalCoordinates(leg, current_pos_global, current_angles);

        // Calculate position error
        Eigen::Vector3d position_error3;
        position_error3 << (local_target.x - current_pos.x),
            (local_target.y - current_pos.y),
            (local_target.z - current_pos.z);

        // Check convergence
        if (position_error3.norm() < tolerance) {
            break;
        }

        // Calculate Jacobian using existing method
        Eigen::Matrix3d jacobian_pos = calculateJacobian(leg, current_angles, local_target);

        // Damped Least Squares (DLS) solution
        Eigen::Matrix3d JJT3 = jacobian_pos * jacobian_pos.transpose();
        Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d damped_inv3 = (JJT3 + dls_coefficient * dls_coefficient * identity3).inverse();
        Eigen::Matrix3d jacobian_inverse3 = jacobian_pos.transpose() * damped_inv3;

        // Calculate joint angle changes
        Eigen::Vector3d angle_delta = jacobian_inverse3 * position_error3;

        // Apply step size limiting for stability
        double max_delta = std::max({std::abs(angle_delta(0)), std::abs(angle_delta(1)), std::abs(angle_delta(2))});
        double step_scale = (max_delta > max_angle_change) ? max_angle_change / max_delta : 1.0;

        // Update joint angles
        current_angles.coxa += angle_delta(0) * step_scale;
        current_angles.femur += angle_delta(1) * step_scale;
        current_angles.tibia += angle_delta(2) * step_scale;

        // Normalize angles to [-PI, PI]
        current_angles.coxa = normalizeAngle(current_angles.coxa);
        current_angles.femur = normalizeAngle(current_angles.femur);
        current_angles.tibia = normalizeAngle(current_angles.tibia);

        // Clamp to joint limits if enabled
        clampJointAngles(current_angles);
    }

    return current_angles;
}

// Helper method to transform global coordinates to local leg coordinates
Point3D RobotModel::transformGlobalToLocalLegCoordinates(int leg, const Point3D &global_target) const {
    const double base_angle_rad = BASE_THETA_OFFSETS[leg];
    double base_x = params.hexagon_radius * cos(base_angle_rad);
    double base_y = params.hexagon_radius * sin(base_angle_rad);

    Point3D local_target;
    double dx = global_target.x - base_x;
    double dy = global_target.y - base_y;
    double angle_rad = -base_angle_rad;
    local_target.x = cos(angle_rad) * dx - sin(angle_rad) * dy;
    local_target.y = sin(angle_rad) * dx + cos(angle_rad) * dy;
    local_target.z = global_target.z;

    return local_target;
}

// Helper method to clamp joint angles to limits
void RobotModel::clampJointAngles(JointAngles &angles) const {
    if (params.ik.clamp_joints) {
        angles.coxa = constrainAngle(angles.coxa, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);
        angles.femur = constrainAngle(angles.femur, femur_angle_limits_rad[0], femur_angle_limits_rad[1]);
        angles.tibia = constrainAngle(angles.tibia, tibia_angle_limits_rad[0], tibia_angle_limits_rad[1]);
    }
}

// OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics
JointAngles RobotModel::inverseKinematicsGlobalCoordinates(int leg, const Point3D &p_target) const {
    // Transform target to leg coordinate system
    Point3D local_target = transformGlobalToLocalLegCoordinates(leg, p_target);

    // Initial guess based on target direction and realistic kinematics
    double coxa_start = atan2(local_target.y, local_target.x);
    coxa_start = constrainAngle(coxa_start, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);

    // Initial estimates for femur and tibia based only on DH model
    double femur_estimate = 0.0f;
    double tibia_estimate = 0.0f;

    JointAngles current_angles(coxa_start, femur_estimate, tibia_estimate);
    clampJointAngles(current_angles);

    return solveIK(leg, local_target, current_angles);
}

JointAngles RobotModel::inverseKinematicsCurrentGlobalCoordinates(int leg, const JointAngles &current_angles,
                                                                  const Point3D &p_target) const {
    // Transform target to local leg coordinates using full OpenSHC Pose-based conversion
    Point3D local_target = transformGlobalToLocalCoordinates(leg, p_target, current_angles);

    JointAngles start = current_angles;
    clampJointAngles(start);

    return solveIK(leg, local_target, start);
}

Point3D RobotModel::forwardKinematicsGlobalCoordinates(int leg_index, const JointAngles &angles) const {
    // Forward kinematics: compute full DH transform chain
    // Use double precision for improved stability
    Eigen::Matrix4d transform = legTransform(leg_index, angles);
    return Point3D{transform(0, 3), transform(1, 3), transform(2, 3)};
}

Point3D RobotModel::getLegBasePosition(int leg_index) const {
    // Calculate base position using DH transform matrix
    // Apply only the base transform (row 0) without joint angles
    Eigen::Matrix4d base_transform = math_utils::dhTransform<double>(
        dh_transforms[leg_index][0][0],  // a0 = hexagon_radius
        dh_transforms[leg_index][0][1],  // alpha0 = 0
        dh_transforms[leg_index][0][2],  // d1 = 0
        dh_transforms[leg_index][0][3]); // theta0 = BASE_THETA_OFFSETS[leg_index]

    // Extract position from the transform matrix
    return Point3D{base_transform(0, 3), base_transform(1, 3), base_transform(2, 3)};
}

double RobotModel::getLegBaseAngleOffset(int leg_index) const {
    // Return the base angle offset for the specified leg
    // This is the theta offset from the DH parameters (BASE_THETA_OFFSETS)
    return dh_transforms[leg_index][0][3];
}

Eigen::Matrix4d RobotModel::legTransform(int leg_index, const JointAngles &q) const {
    // Base transform from DH parameters (body center to leg mount)
    Eigen::Matrix4d T = math_utils::dhTransform<double>(
        dh_transforms[leg_index][0][0],
        dh_transforms[leg_index][0][1],
        dh_transforms[leg_index][0][2],
        dh_transforms[leg_index][0][3]);

    const double joint_rad[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Apply joint transforms.
    // Femur and tibia joints pitch about the Y axis, so rows 2 and 3 use
    // dhTransformY instead of the standard Z-axis version.
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        double a = dh_transforms[leg_index][j][0];      // link length
        double alpha = dh_transforms[leg_index][j][1];  // twist angle
        double d = dh_transforms[leg_index][j][2];      // link offset
        double theta0 = dh_transforms[leg_index][j][3]; // joint offset
        double theta = theta0 + joint_rad[j - 1];       // total joint angle
        if (j >= 2) {
            T *= math_utils::dhTransformY<double>(a, alpha, d, theta);
        } else {
            T *= math_utils::dhTransform<double>(a, alpha, d, theta);
        }
    }

    return T;
}

Eigen::Matrix3d RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &) const {
    // Numerical Jacobian computation using DH-based forward kinematics
    const double delta = JACOBIAN_DELTA;

    Point3D base = forwardKinematicsGlobalCoordinates(leg, q);

    JointAngles qd = q;
    qd.coxa += delta;
    Point3D p_dx = forwardKinematicsGlobalCoordinates(leg, qd);

    qd = q;
    qd.femur += delta;
    Point3D p_dy = forwardKinematicsGlobalCoordinates(leg, qd);

    qd = q;
    qd.tibia += delta;
    Point3D p_dz = forwardKinematicsGlobalCoordinates(leg, qd);

    Eigen::Matrix3d jacobian;
    jacobian.col(0) = Eigen::Vector3d((p_dx.x - base.x) / delta,
                                      (p_dx.y - base.y) / delta,
                                      (p_dx.z - base.z) / delta);
    jacobian.col(1) = Eigen::Vector3d((p_dy.x - base.x) / delta,
                                      (p_dy.y - base.y) / delta,
                                      (p_dy.z - base.z) / delta);
    jacobian.col(2) = Eigen::Vector3d((p_dz.x - base.x) / delta,
                                      (p_dz.y - base.y) / delta,
                                      (p_dz.z - base.z) / delta);

    return jacobian;
}

// Helper method to build DH transforms for a leg
std::vector<Eigen::Matrix4d> RobotModel::buildDHTransforms(int leg, const JointAngles &q) const {
    // Base transform for this leg
    Eigen::Matrix4d T_base = math_utils::dhTransform<double>(
        dh_transforms[leg][0][0],
        dh_transforms[leg][0][1],
        dh_transforms[leg][0][2],
        dh_transforms[leg][0][3]);

    std::vector<Eigen::Matrix4d> transforms(DOF_PER_LEG + 1);
    transforms[0] = T_base;

    const double joint_rad[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    // Build transforms step by step using DH parameters.
    // As above, pitch joints require the Y-axis transform variant.
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        double a = dh_transforms[leg][j][0];      // link length
        double alpha = dh_transforms[leg][j][1];  // twist angle
        double d = dh_transforms[leg][j][2];      // link offset
        double theta0 = dh_transforms[leg][j][3]; // joint offset
        double theta = theta0 + joint_rad[j - 1]; // total joint angle
        if (j >= 2) {
            transforms[j] = transforms[j - 1] *
                            math_utils::dhTransformY<double>(a, alpha, d, theta);
        } else {
            transforms[j] = transforms[j - 1] *
                            math_utils::dhTransform<double>(a, alpha, d, theta);
        }
    }

    return transforms;
}

bool RobotModel::checkJointLimits(int leg_index, const JointAngles &angles) const {
    return (angles.coxa >= coxa_angle_limits_rad[0] && angles.coxa <= coxa_angle_limits_rad[1] &&
            angles.femur >= femur_angle_limits_rad[0] && angles.femur <= femur_angle_limits_rad[1] &&
            angles.tibia >= tibia_angle_limits_rad[0] && angles.tibia <= tibia_angle_limits_rad[1]);
}

double RobotModel::constrainAngle(double angle, double min_angle, double max_angle) const {
    // First normalize angle to [-PI, PI] range to handle wraparound
    double normalized_angle = normalizeAngle(angle);

    // Then clamp to the specified joint limits
    return std::max(min_angle, std::min(max_angle, normalized_angle));
}

double RobotModel::normalizeAngle(double angle_rad) const {
    // Normalize angle to [-PI, PI] range using atan2 trick
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));
    return angle_rad;
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

    const double coxa_step = (coxa_angle_limits_rad[1] - coxa_angle_limits_rad[0]) / resolution;
    const double femur_step = (femur_angle_limits_rad[1] - femur_angle_limits_rad[0]) / resolution;
    const double tibia_step = (tibia_angle_limits_rad[1] - tibia_angle_limits_rad[0]) / resolution;

    // Evaluate the entire workspace of valid joint configurations
    for (int i = 0; i <= resolution; i++) {
        for (int j = 0; j <= resolution; j++) {
            for (int k = 0; k <= resolution; k++) {
                double coxa = coxa_angle_limits_rad[0] + i * coxa_step;
                double femur = femur_angle_limits_rad[0] + j * femur_step;
                double tibia = tibia_angle_limits_rad[0] + k * tibia_step;
                JointAngles q(coxa, femur, tibia);

                // Check that angles are within limits
                if (!checkJointLimits(0, q))
                    continue;

                Point3D pos = forwardKinematicsGlobalCoordinates(0, q);

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

JointAngles RobotModel::calculateTargetFromCurrentPosition(int leg, const JointAngles &current_angles,
                                                           const Pose &current_pose, const Point3D &target_in_current_frame) const {
    // OpenSHC logic: transform target from current pose frame to robot frame
    // This matches OpenSHC's: target_tip_position = model_->getCurrentPose().inverseTransformVector(default_tip_position);
    Point3D target_in_robot_frame = current_pose.inverseTransformVector(target_in_current_frame);

    // Use inverseKinematicsCurrent to calculate joint angles from current position to target
    return inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, target_in_robot_frame);
}

JointAngles RobotModel::calculateTargetFromDefaultStance(int leg, const JointAngles &current_angles,
                                                         const Pose &current_pose, const Pose &default_stance_pose) const {
    // La pose por defecto ya está en el frame del robot, pasar directamente
    return inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, default_stance_pose.position);
}

JointAngles RobotModel::solveIKLocalCoordinates(int leg, const Point3D &global_target,
                                                const JointAngles &current_angles) const {
    // Transform global target to local leg coordinates (OpenSHC-style)
    Point3D local_target = transformGlobalToLocalCoordinates(leg, global_target, current_angles);

    // Solve IK in local coordinates using existing solveIK method
    return solveIK(leg, local_target, current_angles);
}

Point3D RobotModel::transformGlobalToLocalCoordinates(int leg, const Point3D &global_position,
                                                      const JointAngles &current_angles) const {
    // Create pose from global position
    Pose global_pose(global_position, Eigen::Quaterniond::Identity());

    // Transform to local leg coordinates using OpenSHC approach
    Pose local_pose = getPoseLegFrame(leg, current_angles, global_pose);

    return local_pose.position;
}

Point3D RobotModel::transformLocalToGlobalCoordinates(int leg, const Point3D &local_position,
                                                      const JointAngles &current_angles) const {
    // Create pose from local position
    Pose local_pose(local_position, Eigen::Quaterniond::Identity());

    // Transform to global robot coordinates using OpenSHC approach
    Pose global_pose = getPoseRobotFrame(leg, current_angles, local_pose);

    return global_pose.position;
}

double RobotModel::setDesiredTipPoseAndApplyIK(int leg, const Point3D &global_desired_pose,
                                               const JointAngles &current_angles) const {
    // Get current tip pose in global coordinates
    Point3D global_current_pose = forwardKinematicsGlobalCoordinates(leg, current_angles);

    // Calculate position delta in local coordinates using existing method
    Point3D local_position_delta = calculatePositionDeltaLocalCoordinates(leg, global_desired_pose,
                                                                          global_current_pose, current_angles);

    // Check if delta is reasonable (OpenSHC validation)
    if (local_position_delta.norm() > 100.0) { // 100mm sanity check - reasonable max step size
        return 0.0;                            // Failure
    }

    // Transform desired pose to local coordinates for IK solving
    Point3D local_desired_pose = transformGlobalToLocalCoordinates(leg, global_desired_pose, current_angles);

    // Solve IK in local coordinates
    JointAngles new_angles = solveIK(leg, local_desired_pose, current_angles);

    // Validate solution and check accuracy
    if (!checkJointLimits(leg, new_angles)) {
        return 0.0; // Failure
    }

    // Calculate final tip position to check accuracy
    Point3D final_global_pose = forwardKinematicsGlobalCoordinates(leg, new_angles);
    double position_error = math_utils::distance(final_global_pose, global_desired_pose);

    return (position_error <= IK_TOLERANCE) ? 1.0 : 0.0; // Success if error within tolerance
}

Point3D RobotModel::calculatePositionDeltaLocalCoordinates(int leg, const Point3D &global_desired_pose,
                                                           const Point3D &global_current_pose,
                                                           const JointAngles &current_angles) const {
    // Transform both poses to local coordinates (OpenSHC approach)
    Point3D local_desired_pose = transformGlobalToLocalCoordinates(leg, global_desired_pose, current_angles);
    Point3D local_current_pose = transformGlobalToLocalCoordinates(leg, global_current_pose, current_angles);

    // Calculate delta in local coordinates
    return local_desired_pose - local_current_pose;
}

JointAngles RobotModel::estimateInitialAngles(int leg, const Point3D &target_position) const {
    // Transform target to leg coordinate system for analysis
    Point3D local_target = transformGlobalToLocalLegCoordinates(leg, target_position);

    // Method 1: Improved neutral configuration (closer to typical stance)
    // Use angles that are more likely to be in a reasonable stance position
    JointAngles neutral_angles(
        0.0,  // coxa: centered
        -0.2, // femur: slightly negative for typical stance (-11.5 degrees)
        0.2   // tibia: slightly positive for typical stance (11.5 degrees)
    );

    // Method 2: Simple geometric estimation (conservative approach)
    double coxa_estimate = atan2(local_target.y, local_target.x);
    coxa_estimate = constrainAngle(coxa_estimate, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);

    // Use conservative estimates for femur and tibia
    double femur_estimate = -0.2; // Slightly negative stance angle
    double tibia_estimate = 0.2;  // Slightly positive stance angle

    // Only adjust if target is significantly different from neutral stance
    double horizontal_distance = std::sqrt(local_target.x * local_target.x + local_target.y * local_target.y);
    double vertical_distance = -local_target.z;

    if (horizontal_distance > 50.0) { // Only adjust for targets far from base
        // Simple adjustment based on target position
        if (vertical_distance > 50.0) {
            // Target is much lower - need more negative femur
            femur_estimate = -0.4; // About -23 degrees
        } else if (vertical_distance < -50.0) {
            // Target is much higher - need more positive femur
            femur_estimate = 0.0; // About 0 degrees
        }

        if (horizontal_distance > 150.0) {
            // Target is far - need more positive tibia
            tibia_estimate = 0.4; // About 23 degrees
        }
    }

    // Create conservative estimate
    JointAngles conservative_estimate(coxa_estimate, femur_estimate, tibia_estimate);
    clampJointAngles(conservative_estimate);

    // Method 3: Return the conservative estimate as it's more likely to be stable
    // This approach prioritizes stability over precision, which is better for hexapods
    return conservative_estimate;
}
