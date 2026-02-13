#include "robot_model.h"
#include "hexamotion_constants.h"
#include "workspace_analyzer.h"

/**
 * @file robot_model.cpp
 * @brief Implementation of the kinematic robot model.
 */
#include <algorithm>
#include <limits>
#include <math.h>
#include <memory>
#include <stdexcept>
#include <vector>

RobotModel::RobotModel(const Parameters &params)
    : params(params), workspace_analyzer_(nullptr) {

    /** Convert configuration angles from degrees to radians for internal use. */
    /** Keep original parameters in degrees for configuration. */
    for (int i = 0; i < 2; ++i) {
        coxa_angle_limits_rad[i] = math_utils::degreesToRadians(params.coxa_angle_limits[i]);
        femur_angle_limits_rad[i] = math_utils::degreesToRadians(params.femur_angle_limits[i]);
        tibia_angle_limits_rad[i] = math_utils::degreesToRadians(params.tibia_angle_limits[i]);
    }

    max_angular_velocity_rad = math_utils::degreesToRadians(params.max_angular_velocity);
    body_comp_max_tilt_rad = math_utils::degreesToRadians(params.body_comp.max_tilt_deg);

    initializeDH();
}

RobotModel::~RobotModel() {
    /** The destructor must be in the .cpp where WorkspaceAnalyzer is fully defined. */
}

void RobotModel::workspaceAnalyzerInitializer(ComputeConfig config, const ValidationConfig *validation_config) {
    /** Create the WorkspaceAnalyzer only if it does not exist. */
    if (!workspace_analyzer_) {
        if (validation_config) {
            workspace_analyzer_ = std::make_unique<WorkspaceAnalyzer>(*this, config, *validation_config);
        } else {
            /** Use default configuration. */
            ValidationConfig default_config;
            workspace_analyzer_ = std::make_unique<WorkspaceAnalyzer>(*this, config, default_config);
        }
        workspace_analyzer_->initialize();
    }
}

WorkspaceAnalyzer &RobotModel::getWorkspaceAnalyzer() {
    if (!workspace_analyzer_) {
        /** If it has not been initialized, use default configuration. */
        workspaceAnalyzerInitializer();
    }
    return *workspace_analyzer_;
}

const WorkspaceAnalyzer &RobotModel::getWorkspaceAnalyzer() const {
    if (!workspace_analyzer_) {
        throw std::runtime_error("WorkspaceAnalyzer has not been initialized. Call workspaceAnalyzerInitializer() first.");
    }
    return *workspace_analyzer_;
}

Point3D RobotModel::getLegDefaultPosition(int leg_index) const {
    if (leg_index < 0 || leg_index >= NUM_LEGS) {
        /** Return origin for invalid index. */
        return Point3D(0, 0, 0);
    }

    /** Calculate default position using zero joint angles (standing pose). */
    JointAngles zero_angles(0, 0, 0);
    return forwardKinematicsGlobalCoordinates(leg_index, zero_angles);
}

void RobotModel::initializeDH() {

    /** Initialize default DH parameters if custom parameters are not used. */
    /** The analytic model pitches the femur and tibia about the Y axis. */
    /** Replicate that behavior by later applying dhTransformY on rows 2 and 3. */
    if (!params.use_custom_dh_parameters) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            /** Row 0: fixed base. */
            dh_transforms[l][0][0] = params.hexagon_radius;
            dh_transforms[l][0][1] = 0.0f;
            dh_transforms[l][0][2] = 0.0f;
            dh_transforms[l][0][3] = BASE_THETA_OFFSETS[l];

            /** Row 1: yaw servo + coxa. */
            dh_transforms[l][1][0] = params.coxa_length;
            dh_transforms[l][1][1] = 0.0f;
            dh_transforms[l][1][2] = 0.0f;
            dh_transforms[l][1][3] = 0.0f;

            /** Row 2: femur pitch servo. */
            dh_transforms[l][2][0] = params.femur_length;
            dh_transforms[l][2][1] = 0.0f;
            dh_transforms[l][2][2] = 0.0f;
            dh_transforms[l][2][3] = 0.0f;

            /** Row 3: knee pitch servo + tibia. */
            dh_transforms[l][3][0] = 0.0f;
            dh_transforms[l][3][1] = 0.0f;
            dh_transforms[l][3][2] = -params.tibia_length;
            dh_transforms[l][3][3] = 0.0f;
        }
    } else {
        /** Copy custom DH parameters provided in params.dh_parameters. */
        for (int l = 0; l < NUM_LEGS; ++l) {
            for (int j = 0; j < DOF_PER_LEG + 1; ++j) {
                for (int k = 0; k < 4; ++k) {
                    dh_transforms[l][j][k] = params.dh_parameters[l][j][k];
                }
            }
        }
    }
}

/** OpenSHC-style Damped Least Squares (DLS) iterative inverse kinematics. */
JointAngles RobotModel::solveIK(int leg, const Point3D &global_target, JointAngles current,
                                JointAngles current_velocity) const {
    const double tolerance = IK_TOLERANCE;
    const double dls_coefficient = IK_DLS_COEFFICIENT;
    const double max_joint_speed = IK_MAX_JOINT_ANGULAR_SPEED;
    /** Max angle change per iteration. */
    const double max_angle_change = math_utils::degreesToRadians(IK_MAX_ANGLE_STEP);

    for (int iter = 0; iter < params.ik.max_iterations; ++iter) {
        /** OpenSHC parity: compute error in global frame (matches Jacobian frame). */
        /** Previous code transformed FK output through legTransform.inverse(). */
        /** That cancelled out (FK is legTransform * origin), yielding current_pos ~= (0, 0, 0). */
        Point3D current_pos = forwardKinematicsGlobalCoordinates(leg, current);

        /** Calculate position error in global frame. */
        Eigen::Vector3d position_error3;
        position_error3 << (global_target.x - current_pos.x),
            (global_target.y - current_pos.y),
            (global_target.z - current_pos.z);

        /** Check convergence. */
        if (position_error3.norm() < tolerance) {
            break;
        }

        /** Calculate Jacobian (already in global frame via numerical FK perturbation). */
        Eigen::Matrix3d jacobian_pos = calculateJacobian(leg, current, global_target);

        /** Damped Least Squares (DLS) solution. */
        Eigen::Matrix3d JJT3 = jacobian_pos * jacobian_pos.transpose();
        Eigen::Matrix3d identity3 = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d damped_inv3 = (JJT3 + dls_coefficient * dls_coefficient * identity3).inverse();
        Eigen::Matrix3d jacobian_inverse3 = jacobian_pos.transpose() * damped_inv3;

        /** OpenSHC nullspace optimization: joint limit cost function (REF: Autonomous Robots, Fahimi 2008). */
        /** This guides IK toward configurations that keep joints away from limits. */
        double position_limit_cost = 0.0;
        double velocity_limit_cost = 0.0;
        Eigen::Vector3d position_cost_gradient = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity_cost_gradient = Eigen::Vector3d::Zero();
        const double cost_weight = IK_JOINT_LIMIT_COST_WEIGHT;

        /** Calculate cost for each joint approaching limits. */
        double joint_positions[3] = {current.coxa, current.femur, current.tibia};
        double joint_velocities[3] = {current_velocity.coxa, current_velocity.femur, current_velocity.tibia};
        double joint_limits[3][2] = {
            {coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]},
            {femur_angle_limits_rad[0], femur_angle_limits_rad[1]},
            {tibia_angle_limits_rad[0], tibia_angle_limits_rad[1]}};

        for (int j = 0; j < 3; ++j) {
            /** POSITION LIMITS (OpenSHC parity) */
            double joint_range = joint_limits[j][1] - joint_limits[j][0];
            double range_center = joint_limits[j][0] + joint_range / 2.0;
            if (joint_range > 0.0) {
                double normalized_pos = (joint_positions[j] - range_center) / joint_range;
                position_limit_cost += (cost_weight * normalized_pos) * (cost_weight * normalized_pos);
                position_cost_gradient(j) = -(cost_weight * cost_weight * (joint_positions[j] - range_center)) / (joint_range * joint_range);
            }

            /** VELOCITY LIMITS (OpenSHC parity) */
            double joint_velocity_range = 2.0 * max_joint_speed;
            double velocity_range_centre = 0.0;
            double normalized_vel = (joint_velocities[j] - velocity_range_centre) / joint_velocity_range;
            velocity_limit_cost += (cost_weight * normalized_vel) * (cost_weight * normalized_vel);
            velocity_cost_gradient(j) = -(cost_weight * cost_weight * (joint_velocities[j] - velocity_range_centre)) / (joint_velocity_range * joint_velocity_range);
        }

        /** Normalize gradients (OpenSHC parity). */
        if (position_limit_cost > 0.0) {
            position_cost_gradient *= 1.0 / std::sqrt(position_limit_cost);
        }
        if (velocity_limit_cost > 0.0) {
            velocity_cost_gradient *= 1.0 / std::sqrt(velocity_limit_cost);
        }

        /** OpenSHC exact interpolation: interpolate(position, velocity, 0.75). */
        /** Equivalent to 0.25 * position + 0.75 * velocity. */
        Eigen::Vector3d combined_cost_gradient = 0.25 * position_cost_gradient +
                                                 0.75 * velocity_cost_gradient;

        /** Calculate joint angle changes with nullspace projection. */
        /** Primary term: jacobian_inverse * error (achieves target position). */
        /** Nullspace term: (I - J# * J) * gradient (optimizes joint configuration without affecting position). */
        Eigen::Matrix3d nullspace_projector = identity3 - jacobian_inverse3 * jacobian_pos;
        Eigen::Vector3d angle_delta = jacobian_inverse3 * position_error3 + nullspace_projector * combined_cost_gradient;

        /** Apply step size limiting for stability. */
        double max_delta = std::max({std::abs(angle_delta(0)), std::abs(angle_delta(1)), std::abs(angle_delta(2))});
        double step_scale = (max_delta > max_angle_change) ? max_angle_change / max_delta : 1.0;

        /** Update joint angles. */
        current.coxa += angle_delta(0) * step_scale;
        current.femur += angle_delta(1) * step_scale;
        current.tibia += angle_delta(2) * step_scale;

        /** Normalize angles to [-PI, PI]. */
        current.coxa = normalizeAngle(current.coxa);
        current.femur = normalizeAngle(current.femur);
        current.tibia = normalizeAngle(current.tibia);

        /** OpenSHC approach: do not clamp during iterations, let IK converge naturally. */
        /** Clamping will be applied at the end if needed. */
    }

    /** Apply final clamping only after convergence (OpenSHC style). */
    if (params.ik.clamp_joints) {
        clampJointAngles(current);
    }

    return current;
}

/** Helper method to transform global coordinates to local leg coordinates. */
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

/** Helper method to clamp joint angles to limits. */
void RobotModel::clampJointAngles(JointAngles &angles) const {
    if (params.ik.clamp_joints) {
        angles.coxa = constrainAngle(angles.coxa, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);
        angles.femur = constrainAngle(angles.femur, femur_angle_limits_rad[0], femur_angle_limits_rad[1]);
        angles.tibia = constrainAngle(angles.tibia, tibia_angle_limits_rad[0], tibia_angle_limits_rad[1]);
    }
}

void RobotModel::clampToJointLimits(JointAngles &angles) const {
    angles.coxa = constrainAngle(angles.coxa, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);
    angles.femur = constrainAngle(angles.femur, femur_angle_limits_rad[0], femur_angle_limits_rad[1]);
    angles.tibia = constrainAngle(angles.tibia, tibia_angle_limits_rad[0], tibia_angle_limits_rad[1]);
}

/**
 * @brief Solve inverse kinematics from scratch for a global-frame target.
 *
 * Unlike inverseKinematicsCurrentGlobalCoordinates (which refines from the
 * current joint state, mirroring OpenSHC's incremental per-cycle approach),
 * this method computes a full IK solution without requiring a prior joint
 * configuration.  It is intended for situations where no reasonable seed
 * angles are available — e.g. initial stance setup, teleporting a foot to
 * an arbitrary position, or validating workspace reachability — so the
 * solver cannot rely on small incremental deltas.
 *
 * An analytical initial guess is derived from the DH chain geometry
 * (law of cosines for tibia, linear system for femur, atan2 for coxa)
 * and then refined with a DLS iterative solver.  Two coxa candidates
 * (direct and π-flipped) and two tibia solutions (elbow-up / elbow-down)
 * are evaluated to handle backward-folding configurations and joint-limit
 * boundary cases.
 *
 * @param leg   Internal leg index (0–5).
 * @param p     Desired tip position in the global (robot body) frame.
 * @return      Joint angles that place the tip at @p p (clamped to limits
 *              if clamping is enabled).
 */
JointAngles RobotModel::inverseKinematicsGlobalCoordinates(int leg, const Point3D &p) const {
    /** Transform target to leg coordinate system for initial guess only. */
    Point3D local_target = transformGlobalToLocalLegCoordinates(leg, p);

    const double f = params.femur_length;
    const double t = params.tibia_length;
    const double base_angle = BASE_THETA_OFFSETS[leg];
    /** Small tolerance for floating-point boundary comparisons. */
    const double eps = 1e-9;

    /** Coxa from atan2 may point forward when the leg is folded backward (high femur angles). */
    /** Build candidate coxa angles: direct atan2 result and the pi-flipped version. */
    double coxa_raw = atan2(local_target.y, local_target.x);
    double coxa_candidates[2] = {coxa_raw, normalizeAngle(coxa_raw + M_PI)};

    double best_error = std::numeric_limits<double>::max();
    JointAngles best_guess(0.0, 0.0, 0.0);

    for (int c = 0; c < 2; ++c) {
        double coxa_c = constrainAngle(coxa_candidates[c],
                                       coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);
        double total_angle = base_angle + coxa_c;

        /** Femur pivot in global coordinates. */
        double fp_x = params.hexagon_radius * cos(base_angle) + params.coxa_length * cos(total_angle);
        double fp_y = params.hexagon_radius * sin(base_angle) + params.coxa_length * sin(total_angle);

        /** Displacement from femur pivot to target, projected onto coxa radial direction. */
        double gdx = p.x - fp_x;
        double gdy = p.y - fp_y;
        /** Radial. */
        double dx = cos(total_angle) * gdx + sin(total_angle) * gdy;
        /** Vertical (femur pivot z = 0). */
        double dz = p.z;

        double R2 = dx * dx + dz * dz;
        double sin_tibia = (f * f + t * t - R2) / (2.0 * f * t);
        if (sin_tibia < -1.0 - eps || sin_tibia > 1.0 + eps)
            continue;
        sin_tibia = std::max(-1.0, std::min(1.0, sin_tibia));

        /** Try both tibia solutions (elbow up / elbow down). */
        double tibia_sols[2] = {asin(sin_tibia), M_PI - asin(sin_tibia)};
        for (int s = 0; s < 2; ++s) {
            double tibia_e = tibia_sols[s];
            double P = f - t * sin(tibia_e);
            double Q = t * cos(tibia_e);
            double femur_e = atan2(-(Q * dx + P * dz), P * dx - Q * dz);

            /** Skip if clearly outside joint limits (with floating-point tolerance). */
            if (femur_e < femur_angle_limits_rad[0] - eps || femur_e > femur_angle_limits_rad[1] + eps ||
                tibia_e < tibia_angle_limits_rad[0] - eps || tibia_e > tibia_angle_limits_rad[1] + eps)
                continue;

            /** Clamp to exact limits before FK evaluation. */
            femur_e = std::max(femur_angle_limits_rad[0], std::min(femur_angle_limits_rad[1], femur_e));
            tibia_e = std::max(tibia_angle_limits_rad[0], std::min(tibia_angle_limits_rad[1], tibia_e));

            /** Evaluate FK error for this candidate. */
            JointAngles candidate(coxa_c, femur_e, tibia_e);
            Point3D fk = forwardKinematicsGlobalCoordinates(leg, candidate);
            double err = (fk.x - p.x) * (fk.x - p.x) +
                         (fk.y - p.y) * (fk.y - p.y) +
                         (fk.z - p.z) * (fk.z - p.z);
            if (err < best_error) {
                best_error = err;
                best_guess = candidate;
            }
        }
    }

    /** If no analytical candidate was found, fall back to clamped atan2. */
    if (best_error == std::numeric_limits<double>::max()) {
        double coxa_start = constrainAngle(coxa_raw, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);
        best_guess = JointAngles(coxa_start, 0.0, 0.0);
        clampJointAngles(best_guess);
    }

    /** Pass global target directly; solveIK works in global frame (matching Jacobian). */
    return solveIK(leg, p, best_guess);
}

JointAngles RobotModel::inverseKinematicsCurrentGlobalCoordinates(int leg, const JointAngles &current,
                                                                  const Point3D &target) const {
    /** Pass global target directly; solveIK works in global frame (matching Jacobian). */
    JointAngles start = current;
    clampJointAngles(start);

    return solveIK(leg, target, start);
}

Point3D RobotModel::forwardKinematicsGlobalCoordinates(int leg, const JointAngles &q) const {
    /** Forward kinematics: compute full DH transform chain. */
    /** Use double precision for improved stability. */
    Eigen::Matrix4d transform = legTransform(leg, q);
    return Point3D{transform(0, 3), transform(1, 3), transform(2, 3)};
}

Point3D RobotModel::getLegBasePosition(int leg_index) const {
    /** Calculate base position using DH transform matrix. */
    /** Apply only the base transform (row 0) without joint angles. */
    Eigen::Matrix4d base_transform = math_utils::dhTransform<double>(
        dh_transforms[leg_index][0][0],
        dh_transforms[leg_index][0][1],
        dh_transforms[leg_index][0][2],
        dh_transforms[leg_index][0][3]);

    /** Extract position from the transform matrix. */
    return Point3D{base_transform(0, 3), base_transform(1, 3), base_transform(2, 3)};
}

double RobotModel::getLegBaseAngleOffset(int leg_index) const {
    /** Return the base angle offset for the specified leg. */
    /** This is the theta offset from the DH parameters (BASE_THETA_OFFSETS). */
    return dh_transforms[leg_index][0][3];
}

Eigen::Matrix4d RobotModel::legTransform(int leg, const JointAngles &q) const {
    /** Base transform from DH parameters (body center to leg mount). */
    Eigen::Matrix4d T = math_utils::dhTransform<double>(
        dh_transforms[leg][0][0],
        dh_transforms[leg][0][1],
        dh_transforms[leg][0][2],
        dh_transforms[leg][0][3]);

    const double joint_rad[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    /** Apply joint transforms. */
    /** Femur and tibia joints pitch about the Y axis, so rows 2 and 3 use dhTransformY. */
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        /** Link length. */
        double a = dh_transforms[leg][j][0];
        /** Twist angle. */
        double alpha = dh_transforms[leg][j][1];
        /** Link offset. */
        double d = dh_transforms[leg][j][2];
        /** Joint offset. */
        double theta0 = dh_transforms[leg][j][3];
        /** Total joint angle. */
        double theta = theta0 + joint_rad[j - 1];
        if (j >= 2) {
            T *= math_utils::dhTransformY<double>(a, alpha, d, theta);
        } else {
            T *= math_utils::dhTransform<double>(a, alpha, d, theta);
        }
    }

    return T;
}

Eigen::Matrix3d RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &) const {
    /** Numerical Jacobian computation using DH-based forward kinematics. */
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

/** Helper method to build DH transforms for a leg. */
std::vector<Eigen::Matrix4d> RobotModel::buildDHTransforms(int leg, const JointAngles &q) const {
    /** Base transform for this leg. */
    Eigen::Matrix4d T_base = math_utils::dhTransform<double>(
        dh_transforms[leg][0][0],
        dh_transforms[leg][0][1],
        dh_transforms[leg][0][2],
        dh_transforms[leg][0][3]);

    std::vector<Eigen::Matrix4d> transforms(DOF_PER_LEG + 1);
    transforms[0] = T_base;

    const double joint_rad[DOF_PER_LEG] = {q.coxa, q.femur, q.tibia};

    /** Build transforms step by step using DH parameters. */
    /** Pitch joints require the Y-axis transform variant. */
    for (int j = 1; j <= DOF_PER_LEG; ++j) {
        /** Link length. */
        double a = dh_transforms[leg][j][0];
        /** Twist angle. */
        double alpha = dh_transforms[leg][j][1];
        /** Link offset. */
        double d = dh_transforms[leg][j][2];
        /** Joint offset. */
        double theta0 = dh_transforms[leg][j][3];
        /** Total joint angle. */
        double theta = theta0 + joint_rad[j - 1];
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
    /** First normalize angle to [-PI, PI] range to handle wraparound. */
    double normalized_angle = normalizeAngle(angle);

    /** Then clamp to the specified joint limits. */
    return std::max(min_angle, std::min(max_angle, normalized_angle));
}

double RobotModel::normalizeAngle(double angle_rad) const {
    /** Normalize angle to [-PI, PI] range using atan2 trick. */
    angle_rad = atan2(sin(angle_rad), cos(angle_rad));
    return angle_rad;
}

bool RobotModel::validate() const {
    return (params.hexagon_radius > 0 && params.coxa_length > 0 && params.femur_length > 0 && params.tibia_length > 0 &&
            params.robot_height > 0 && params.time_delta > 0);
}

std::pair<double, double> RobotModel::calculateHeightRange() const {
    double min_h = std::numeric_limits<double>::max();
    double max_h = -std::numeric_limits<double>::max();

    /** Workspace analysis: discretize the joint configuration space. */
    /** Based on "Introduction to Robotics" (Craig) and "Robotics: Modelling, Planning and Control" (Siciliano). */
    const double coxa_step = (coxa_angle_limits_rad[1] - coxa_angle_limits_rad[0]) / WORKSPACE_RESOLUTION;
    const double femur_step = (femur_angle_limits_rad[1] - femur_angle_limits_rad[0]) / WORKSPACE_RESOLUTION;
    const double tibia_step = (tibia_angle_limits_rad[1] - tibia_angle_limits_rad[0]) / WORKSPACE_RESOLUTION;

    /** Evaluate the entire workspace of valid joint configurations. */
    for (int i = 0; i <= WORKSPACE_RESOLUTION; i++) {
        for (int j = 0; j <= WORKSPACE_RESOLUTION; j++) {
            for (int k = 0; k <= WORKSPACE_RESOLUTION; k++) {
                double coxa = coxa_angle_limits_rad[0] + i * coxa_step;
                double femur = femur_angle_limits_rad[0] + j * femur_step;
                double tibia = tibia_angle_limits_rad[0] + k * tibia_step;
                JointAngles q(coxa, femur, tibia);

                /** Check that angles are within limits. */
                if (!checkJointLimits(0, q))
                    continue;

                Point3D pos = forwardKinematicsGlobalCoordinates(0, q);

                /** Calculate body height considering the robot's physical offset. */
                /** pos.z is negative when the leg is below the body. */
                double height = -pos.z + params.height_offset;

                /** Only consider physically valid heights (positive). */
                if (height > 0) {
                    min_h = std::min(min_h, height);
                    max_h = std::max(max_h, height);
                }
            }
        }
    }

    /** If no valid configurations were found, indicates parameter error. */
    if (min_h == std::numeric_limits<double>::max()) {
        /** Return values indicating error; inconsistent robot parameters. */
        return {-1.0f, -1.0f};
    }

    return {min_h, max_h};
}

Pose RobotModel::getPoseRobotFrame(int leg_index, const JointAngles &joint_angles, const Pose &leg_frame_pose) const {
    /** Get the full transform from robot frame to leg frame. */
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    /** Transform the leg frame pose to robot frame. */
    return leg_frame_pose.transform(transform);
}

Pose RobotModel::getPoseLegFrame(int leg_index, const JointAngles &joint_angles, const Pose &robot_frame_pose) const {
    /** Get the full transform from robot frame to leg frame. */
    Eigen::Matrix4d transform = legTransform(leg_index, joint_angles);

    /** Transform the robot frame pose to leg frame (inverse transform). */
    return robot_frame_pose.transform(transform.inverse());
}

double RobotModel::getLegReach() const {
    /** Maximum reach is femur + tibia lengths (coxa only provides lateral offset). */
    /** The coxa rotates around Z-axis and does not extend the radial reach. */
    return params.femur_length + params.tibia_length;
}

double RobotModel::computeStandingHorizontalReach(const Parameters &p) {
    /** Reuse shared femur angle computation (no duplication of trig logic). */
    bool ok = false;
    double femur_angle = 0.0;
    /** Internal lambda replicating height feasibility logic (shared with angle solver). */
    auto computeFemur = [&](double target_height_mm, bool &valid) -> double {
        valid = false;
        if (p.femur_length <= 0.0 || p.tibia_length <= 0.0 || p.coxa_length < 0.0)
            return 0.0;
        double min_h = std::max(0.0, p.tibia_length - p.femur_length);
        double max_h = p.tibia_length + p.femur_length;
        if (target_height_mm < min_h || target_height_mm > max_h)
            /** Invalid. */
            return 0.0;
        /** sin(femur). */
        double sin_theta = (target_height_mm - p.tibia_length) / p.femur_length;
        sin_theta = math_utils::clamp(sin_theta, -1.0, 1.0);
        valid = true;
        return std::asin(sin_theta);
    };
    femur_angle = computeFemur(p.standing_height, ok);
    if (!ok) {
        /** Conservative fallback. */
        return p.coxa_length;
    }
    double horizontal_proj = p.femur_length * std::cos(femur_angle);
    return p.coxa_length + horizontal_proj;
}

double RobotModel::getStandingHorizontalReach() const {
    return computeStandingHorizontalReach(params);
}

CalculatedServoAngles RobotModel::calculateServoAnglesForHeight(double target_height_mm, const Parameters &params) {
    CalculatedServoAngles result{0.0, 0.0, 0.0, false};
    /**
     * Based on analytic_robot_model.cpp leg transform:
     * T = T_base * R_coxa * T_coxa * R_femur * T_femur * R_tibia * T_tibia
     *
     * For leg height calculation with coxa = 0 degrees (radial stance):
     * - T_base: hexagon_radius in XY plane (Z = 0)
     * - R_coxa: rotation around Z axis (coxa = 0 degrees)
     * - T_coxa: translation along X axis (coxa_length)
     * - R_femur: rotation around Y axis (femur angle)
     * - T_femur: translation along X axis (femur_length)
     * - R_tibia: rotation around Y axis (tibia angle)
     * - T_tibia: translation along Z axis (-tibia_length)
     *
     * With coxa = 0 degrees, the Z component of foot position is:
     * Z = -femur_length * sin(femur_angle) - tibia_length * cos(femur_angle + tibia_angle)
     *
     * For standing pose, we want tibia to be vertical (pointing down):
     * femur_angle + tibia_angle = 0 degrees (tibia points straight down)
     * Therefore: tibia_angle = -femur_angle
     *
     * Substituting:
     * Z = -femur_length * sin(femur_angle) - tibia_length * cos(0 degrees)
     * Z = -femur_length * sin(femur_angle) - tibia_length
     *
     * Solving for femur_angle:
     * target_height = -femur_length * sin(femur_angle) - tibia_length
     * sin(femur_angle) = -(target_height + tibia_length) / femur_length
     */

    /** Convert to signed Z frame convention used in prior derivation. */
    double target_z = -target_height_mm;
    double sin_femur = -(target_z + params.tibia_length) / params.femur_length;
    if (sin_femur < -1.0 || sin_femur > 1.0)
        /** Impossible. */
        return result;
    double femur_rad = std::asin(sin_femur);
    /** Keeps tibia vertical. */
    double tibia_rad = -femur_rad;

    double femur_deg = math_utils::radiansToDegrees(femur_rad);
    double tibia_deg = math_utils::radiansToDegrees(tibia_rad);
    if (femur_deg < params.femur_angle_limits[0] || femur_deg > params.femur_angle_limits[1])
        return result;
    if (tibia_deg < params.tibia_angle_limits[0] || tibia_deg > params.tibia_angle_limits[1])
        return result;
    result.coxa = 0.0;
    result.femur = femur_rad;
    result.tibia = tibia_rad;
    result.valid = true;
    return result;
}

double RobotModel::getDefaultHeightOffset() const {
    /** Return the default height offset used in the robot model. */
    return params.default_height_offset;
}

JointAngles RobotModel::calculateTargetFromCurrentPosition(int leg, const JointAngles &current_angles,
                                                           const Pose &current_pose, const Point3D &target_in_current_frame) const {
    /** OpenSHC logic: transform target from current pose frame to robot frame. */
    /** This matches OpenSHC's: target_tip_position = model_->getCurrentPose().inverseTransformVector(default_tip_position). */
    Point3D target_in_robot_frame = current_pose.inverseTransformVector(target_in_current_frame);

    /** Use inverseKinematicsCurrent to calculate joint angles from current position to target. */
    return inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, target_in_robot_frame);
}

JointAngles RobotModel::calculateTargetFromDefaultStance(int leg, const JointAngles &current_angles,
                                                         const Pose &current_pose, const Pose &default_stance_pose) const {
    /** The default pose is already in the robot frame, pass it directly. */
    return inverseKinematicsCurrentGlobalCoordinates(leg, current_angles, default_stance_pose.position);
}

JointAngles RobotModel::solveIKLocalCoordinates(int leg, const Point3D &global_target,
                                                const JointAngles &current_angles) const {
    /** solveIK now works in global frame; pass global target directly. */
    return solveIK(leg, global_target, current_angles);
}

Point3D RobotModel::transformGlobalToLocalCoordinates(int leg, const Point3D &global_position,
                                                      const JointAngles &current_angles) const {
    /** Create pose from global position. */
    Pose global_pose(global_position, Eigen::Quaterniond::Identity());

    /** Transform to local leg coordinates using OpenSHC approach. */
    Pose local_pose = getPoseLegFrame(leg, current_angles, global_pose);

    return local_pose.position;
}

Point3D RobotModel::transformLocalToGlobalCoordinates(int leg, const Point3D &local_position,
                                                      const JointAngles &current_angles) const {
    /** Create pose from local position. */
    Pose local_pose(local_position, Eigen::Quaterniond::Identity());

    /** Transform to global robot coordinates using OpenSHC approach. */
    Pose global_pose = getPoseRobotFrame(leg, current_angles, local_pose);

    return global_pose.position;
}

JointAngles RobotModel::estimateInitialAngles(int leg, const Point3D &target_position) const {
    /** Transform target to leg coordinate system for analysis. */
    Point3D local_target = transformGlobalToLocalLegCoordinates(leg, target_position);

    /** Method 1: improved neutral configuration (closer to typical stance). */
    /** Use angles that are more likely to be in a reasonable stance position. */
    JointAngles neutral_angles(
        0.0,
        -0.2,
        0.2);

    /** Method 2: simple geometric estimation (conservative approach). */
    double coxa_estimate = atan2(local_target.y, local_target.x);
    coxa_estimate = constrainAngle(coxa_estimate, coxa_angle_limits_rad[0], coxa_angle_limits_rad[1]);

    /** Use conservative estimates for femur and tibia. */
    double femur_estimate = -0.2;
    double tibia_estimate = 0.2;

    /** Only adjust if target is significantly different from neutral stance. */
    double horizontal_distance = std::sqrt(local_target.x * local_target.x + local_target.y * local_target.y);

    if (horizontal_distance > 50.0) {
        /** Only adjust for targets far from base. */
        double vertical_offset = -local_target.z;
        /** Simple adjustment based on target position. */
        if (vertical_offset > 50.0) {
            /** Target is much lower; need more negative femur. */
            femur_estimate = -0.4;
        } else if (vertical_offset < -50.0) {
            /** Target is much higher; need more positive femur. */
            femur_estimate = 0.0;
        }

        if (horizontal_distance > 150.0) {
            /** Target is far; need more positive tibia. */
            tibia_estimate = 0.4;
        }
    }

    /** Create conservative estimate. */
    JointAngles conservative_estimate(coxa_estimate, femur_estimate, tibia_estimate);
    clampJointAngles(conservative_estimate);

    /** Method 3: return the conservative estimate as it is more likely to be stable. */
    /** This approach prioritizes stability over precision. */
    return conservative_estimate;
}

Point3D RobotModel::makeReachable(int leg_index, const Point3D &reference_tip_position) const {

    /** Ensure that the workspace is generated (equivalent to OpenSHC's generateWorkspace()). */
    /** Use const_cast because the method is const but needs to modify workspace_analyzer. */
    const_cast<RobotModel *>(this)->getWorkspaceAnalyzer().generateWorkspace();

    /** The height used to query the workplane must consider the physical offset. */
    double workspace_query_height = reference_tip_position.z;

    /** Get the workplane for the target position's adjusted height. */
    auto workplane = getWorkspaceAnalyzer().getWorkplane(leg_index, workspace_query_height);

    if (!workplane.empty()) {
        /** Convert the position to polar coordinates relative to the leg base. */
        Point3D leg_base = getLegBasePosition(leg_index);
        Point3D relative_pos = reference_tip_position - leg_base;

        /** Calculate bearing (angle) and radius. */
        double bearing_rad = atan2(relative_pos.y, relative_pos.x);
        double bearing_deg = math_utils::radiansToDegrees(bearing_rad);

        /** Normalize bearing to [0, 360). */
        if (bearing_deg < 0)
            bearing_deg += 360.0;

        double requested_radius = sqrt(relative_pos.x * relative_pos.x + relative_pos.y * relative_pos.y);

        /** Find the maximum allowed radius in the workplane for this bearing. */
        double max_radius = 0.0;

        /** Interpolation between adjacent bearings in the workplane. */
        int bearing_int = static_cast<int>(bearing_deg);
        auto it_current = workplane.find(bearing_int);
        auto it_next = workplane.find((bearing_int + 1) % 360);

        if (it_current != workplane.end()) {
            max_radius = it_current->second;

            /** Linear interpolation if we have the next bearing. */
            if (it_next != workplane.end()) {
                double fraction = bearing_deg - bearing_int;
                max_radius = it_current->second * (1.0 - fraction) + it_next->second * fraction;
            }
        } else {
            /** If we do not have exact data, search for nearby bearings. */
            double min_bearing_diff = 360.0;
            for (const auto &bearing_pair : workplane) {
                double diff = std::min(std::abs(bearing_deg - bearing_pair.first),
                                       360.0 - std::abs(bearing_deg - bearing_pair.first));
                if (diff < min_bearing_diff) {
                    min_bearing_diff = diff;
                    max_radius = bearing_pair.second;
                }
            }
        }

        /** If the requested position is outside the workspace, constrain it. */
        if (requested_radius > max_radius && max_radius > 0.0) {
            double scale_factor = max_radius / requested_radius;
            Point3D constrained_relative = relative_pos * scale_factor;

            /** Keep the original height considering the physical reference. */
            constrained_relative.z = relative_pos.z;

            return leg_base + constrained_relative;
        }

        /** The position is already within the workspace. */
        return reference_tip_position;
    }

    /** If the workplane is empty, use basic geometric constraint. */
    /** This should only occur in exceptional cases. */
    Point3D leg_base = getLegBasePosition(leg_index);
    Point3D target_vector = reference_tip_position - leg_base;
    double distance_to_target = target_vector.norm();

    double max_reach = params.femur_length + params.tibia_length;
    /** 95% of the maximum reach. */
    double safe_max_reach = max_reach * 0.95;

    if (distance_to_target > safe_max_reach) {
        Point3D safe_direction = target_vector / distance_to_target;
        Point3D safe_position = leg_base + safe_direction * safe_max_reach;
        /** Keep the original height considering that the workspace already includes the physical offset. */
        safe_position.z = reference_tip_position.z;
        return safe_position;
    }

    return reference_tip_position;
}

/** Advanced IK implementation. */

JointAngles RobotModel::applyAdvancedIK(int leg, const Point3D &current_tip_pose, const Point3D &desired_tip_pose,
                                        const JointAngles &current_angles, double time_delta) const {
    /** Following OpenSHC pattern exactly. */
    /** Calculate position delta in global coordinates. */
    Eigen::Vector3d position_delta;
    position_delta << (desired_tip_pose.x - current_tip_pose.x),
        (desired_tip_pose.y - current_tip_pose.y),
        (desired_tip_pose.z - current_tip_pose.z);

    /** Create 6D delta vector (position only). */
    Eigen::MatrixXd delta = Eigen::Matrix<double, 6, 1>::Zero();
    delta(0) = position_delta[0];
    delta(1) = position_delta[1];
    delta(2) = position_delta[2];

    /** Get basic joint delta from DLS method (like OpenSHC's jacobian_inverse * delta). */
    Eigen::Vector3d joint_delta = solveDeltaIK(leg, delta, current_angles);

    /** Calculate joint velocities for cost gradient (estimate from time_delta). */
    Eigen::Vector3d joint_velocities = Eigen::Vector3d::Zero();
    if (time_delta > 0.0) {
        joint_velocities = joint_delta / time_delta;
    }

    /** Apply OpenSHC joint limit cost gradient. */
    /** Matches: return jacobian_inverse * delta + (identity - jacobian_inverse * j) * combined_cost_gradient. */
    Eigen::Vector3d cost_gradient = calculateJointLimitCostGradient(current_angles, joint_velocities, leg);

    /** Calculate Jacobian for nullspace projection (OpenSHC approach). */
    Eigen::Matrix3d jacobian_pos = calculateJacobian(leg, current_angles, Point3D(0, 0, 0));
    const double dls_coeff = IK_DLS_COEFFICIENT;
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d JJT = jacobian_pos * jacobian_pos.transpose();
    Eigen::Matrix3d jacobian_inverse = jacobian_pos.transpose() *
                                       (JJT + dls_coeff * dls_coeff * identity).inverse();

    /** Apply nullspace projection: (I - J^+ * J) * cost_gradient. */
    Eigen::Matrix3d nullspace_projector = identity - jacobian_inverse * jacobian_pos;
    Eigen::Vector3d nullspace_motion = nullspace_projector * cost_gradient;

    /** Combine primary motion with nullspace motion (OpenSHC approach). */
    joint_delta += nullspace_motion;

    /** Apply joint angle changes to current configuration. */
    JointAngles new_angles = current_angles;
    new_angles.coxa += joint_delta(0);
    new_angles.femur += joint_delta(1);
    new_angles.tibia += joint_delta(2);

    /** Normalize angles to [-PI, PI]. */
    new_angles.coxa = normalizeAngle(new_angles.coxa);
    new_angles.femur = normalizeAngle(new_angles.femur);
    new_angles.tibia = normalizeAngle(new_angles.tibia);

    /** Apply joint limits. */
    if (params.ik.clamp_joints) {
        clampJointAngles(new_angles);
    }

    return new_angles;
}

Eigen::Vector3d RobotModel::solveDeltaIK(int leg, const Eigen::MatrixXd &delta, const JointAngles &current_angles) const {
    /** Core IK method: exact replication for 3DOF case. */
    /** Calculate Jacobian. */
    Eigen::Matrix3d jacobian_pos = calculateJacobian(leg, current_angles, Point3D(0, 0, 0));

    /** DLS method. */
    const double dls_coeff = IK_DLS_COEFFICIENT;
    Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d JJT = jacobian_pos * jacobian_pos.transpose();
    Eigen::Matrix3d jacobian_inverse = jacobian_pos.transpose() *
                                       (JJT + dls_coeff * dls_coeff * identity).inverse();

    /** Extract position part of delta (first 3 elements). */
    Eigen::Vector3d position_delta = delta.block<3, 1>(0, 0);

    return jacobian_inverse * position_delta;
}
Eigen::Vector3d RobotModel::calculateJointLimitCostGradient(const JointAngles &current_angles,
                                                            const Eigen::Vector3d &joint_velocities, int /*leg*/) const {
    /** OpenSHC-style joint limit cost gradient (position + velocity). */
    const double cost_weight = IK_JOINT_LIMIT_COST_WEIGHT;

    Eigen::Vector3d position_cost_gradient = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_cost_gradient = Eigen::Vector3d::Zero();

    double position_limit_cost = 0.0;
    double velocity_limit_cost = 0.0;

    /** Joint data arrays for iteration. */
    double joint_positions[3] = {current_angles.coxa, current_angles.femur, current_angles.tibia};
    double joint_limits_min[3] = {coxa_angle_limits_rad[0], femur_angle_limits_rad[0], tibia_angle_limits_rad[0]};
    double joint_limits_max[3] = {coxa_angle_limits_rad[1], femur_angle_limits_rad[1], tibia_angle_limits_rad[1]};
    /** rad/s (placeholder typical values). */
    double max_velocities[3] = {3.0, 3.0, 3.0};

    for (int i = 0; i < 3; ++i) {
        /** Position limit contribution. */
        double joint_position_range = joint_limits_max[i] - joint_limits_min[i];
        if (joint_position_range > 0.0) {
            double position_range_centre = joint_limits_min[i] + joint_position_range * 0.5;
            double pos_term = cost_weight * (joint_positions[i] - position_range_centre) / joint_position_range;
            position_limit_cost += pos_term * pos_term;
            position_cost_gradient[i] = -cost_weight * cost_weight * (joint_positions[i] - position_range_centre) /
                                        (joint_position_range * joint_position_range);
        }

        /** Velocity limit contribution. */
        double joint_velocity_range = 2.0 * max_velocities[i];
        if (joint_velocity_range > 0.0) {
            /** Centre = 0. */
            double vel_term = cost_weight * (joint_velocities[i]) / joint_velocity_range;
            velocity_limit_cost += vel_term * vel_term;
            velocity_cost_gradient[i] = -cost_weight * cost_weight * (joint_velocities[i]) /
                                        (joint_velocity_range * joint_velocity_range);
        }
    }

    /** Normalize gradients to prevent scale explosion. */
    if (position_limit_cost > 0.0) {
        position_cost_gradient /= std::sqrt(position_limit_cost);
    }
    if (velocity_limit_cost > 0.0) {
        velocity_cost_gradient /= std::sqrt(velocity_limit_cost);
    }

    /** OpenSHC interpolation parity: 0.25 * position + 0.75 * velocity. */
    return 0.25 * position_cost_gradient + 0.75 * velocity_cost_gradient;
}

std::string RobotModel::gaitTypeToString(GaitType gait_type) {
    switch (gait_type) {
    case WAVE_GAIT:
        return "wave_gait";
    case TRIPOD_GAIT:
        return "tripod_gait";
    case RIPPLE_GAIT:
        return "ripple_gait";
    case METACHRONAL_GAIT:
        return "metachronal_gait";
    case ADAPTIVE_GAIT:
        return "adaptive_gait";
    case NO_GAIT:
    default:
        return "no_gait";
    }
}

GaitType RobotModel::stringToGaitType(const std::string &gait_name) {
    if (gait_name == "wave_gait") {
        return WAVE_GAIT;
    } else if (gait_name == "tripod_gait") {
        return TRIPOD_GAIT;
    } else if (gait_name == "ripple_gait") {
        return RIPPLE_GAIT;
    } else if (gait_name == "metachronal_gait") {
        return METACHRONAL_GAIT;
    } else if (gait_name == "adaptive_gait") {
        return ADAPTIVE_GAIT;
    } else {
        return NO_GAIT;
    }
}

/** End of file. */
