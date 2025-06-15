#include "HexaModel.h"
#include <limits>
#include <math.h>
#include <vector>

RobotModel::RobotModel(const Parameters &p) : params(p) {
    initializeDH();
}

void RobotModel::initializeDH() {
    bool custom = false;
    // Check if custom DH parameters are provided
    for (int l = 0; l < NUM_LEGS; ++l) {
        for (int j = 0; j < DOF_PER_LEG; ++j) {
            for (int k = 0; k < 4; ++k) {
                dh_transforms[l][j][k] = params.dh_parameters[l][j][k];
                if (params.dh_parameters[l][j][k] != 0.0f)
                    custom = true;
            }
        }
    }

    // Use default DH parameters if no custom parameters provided
    if (!custom) {
        for (int l = 0; l < NUM_LEGS; ++l) {
            // Joint 1 (coxa): vertical axis rotation
            dh_transforms[l][0][0] = 0.0f;   // a0 - link length
            dh_transforms[l][0][1] = -90.0f; // alpha0 - rotate to femur axis
            dh_transforms[l][0][2] = 0.0f;   // d1 - link offset
            dh_transforms[l][0][3] = 0.0f;   // theta1 offset - joint angle offset

            // Joint 2 (femur): pitch axis rotation
            dh_transforms[l][1][0] = params.coxa_length; // a1 - translation along rotated x
            dh_transforms[l][1][1] = 0.0f;               // alpha1 - twist angle
            dh_transforms[l][1][2] = 0.0f;               // d2 - link offset
            dh_transforms[l][1][3] = 0.0f;               // theta2 offset - joint angle offset

            // Joint 3 (tibia): pitch axis rotation
            dh_transforms[l][2][0] = params.femur_length; // a2 - translation along femur
            dh_transforms[l][2][1] = 0.0f;                // alpha2 - twist angle
            dh_transforms[l][2][2] = 0.0f;                // d3 - link offset
            dh_transforms[l][2][3] = 0.0f;                // theta3 offset - joint angle offset
        }
    }
}

// Damped Least Squares (DLS) iterative inverse kinematics
// Based on CSIRO syropod_highlevel_controller implementation
JointAngles RobotModel::inverseKinematics(int leg, const Point3D &p_target) {

    const float DLS_COEFFICIENT = 0.02f; // Damping factor for numerical stability
    const float IK_TOLERANCE = 0.005f;   // 5mm tolerance for convergence
    const int MAX_ITERATIONS = 75;       // Increased iterations for better convergence

    // Transform target to leg coordinate system
    const float base_angle_deg = leg * 60.0f;
    float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle_deg));
    float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle_deg));

    Point3D local_target;
    local_target.x = p_target.x - base_x;
    local_target.y = p_target.y - base_y;
    local_target.z = p_target.z;

    // Quick workspace check using centralized reachability function
    float max_reach = params.coxa_length + params.femur_length + params.tibia_length;
    float min_reach = std::abs(params.femur_length - params.tibia_length);

    // Check workspace bounds using centralized function (allow margin for numerical precision)
    if (!math_utils::isPointReachable(local_target, min_reach * 0.9f, max_reach * 1.02f)) {
        float coxa_angle = atan2(local_target.y, local_target.x) * 180.0f / M_PI;
        coxa_angle = constrainAngle(coxa_angle, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

        // Determine if target is too far or too close
        float target_distance = math_utils::magnitude(local_target);
        if (target_distance > max_reach * 1.02f) {
            // Target too far - return extended pose within joint limits
            return JointAngles(coxa_angle, -45.0f, 60.0f); // Constrained extended pose
        } else {
            // Target too close - return retracted pose within joint limits
            return JointAngles(coxa_angle, 30.0f, -60.0f); // Constrained retracted pose
        }
    }

    /*
     * MULTIPLE STARTING CONFIGURATIONS APPROACH:
     *
     * This implementation differs from the original CSIRO syropod_highlevel_controller
     * by using multiple starting configurations to improve convergence robustness.
     *
     * RATIONALE:
     * - The original CSIRO implementation uses a single initial guess for DLS iteration
     * - For challenging targets (near singularities, workspace boundaries, multiple solutions),
     *   a single starting point may fail to converge or converge to suboptimal solutions
     * - This enhanced approach tries up to 5 different starting configurations to find
     *   the best solution within joint limits
     *
     * CONFIGURATIONS TRIED:
     * 1. Target-oriented pose (-45°, 60°) - typical walking configuration
     * 2. High pose (30°, -60°) - leg raised up
     * 3. Mid pose (-30°, 45°) - intermediate position
     * 4. Straight pose (0°, 0°) - fully extended
     * 5. Extended pose (-60°, 80°) - maximum reach
     *
     * DISABLE WITH: Set params.ik.use_multiple_starts = false to use original CSIRO behavior
     */

    // Try multiple starting positions for challenging cases
    JointAngles best_result;
    float best_error = std::numeric_limits<float>::max();

    // Define starting configurations based on user preference
    std::vector<JointAngles> starting_configs;
    float coxa_start = atan2(local_target.y, local_target.x) * 180.0f / M_PI;
    coxa_start = constrainAngle(coxa_start, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);

    if (params.ik.use_multiple_starts) {
        // Enhanced approach: Multiple starting configurations for better convergence
        starting_configs.push_back(JointAngles(coxa_start, -45.0f, 60.0f)); // Primary guess
        starting_configs.push_back(JointAngles(coxa_start, 30.0f, -60.0f)); // High pose
        starting_configs.push_back(JointAngles(coxa_start, -30.0f, 45.0f)); // Mid pose
        starting_configs.push_back(JointAngles(coxa_start, 0.0f, 0.0f));    // Straight pose
        starting_configs.push_back(JointAngles(coxa_start, -60.0f, 80.0f)); // Extended pose
    } else {
        // Original CSIRO approach: Single starting configuration
        starting_configs.push_back(JointAngles(coxa_start, -45.0f, 60.0f));
    }

    // Try each starting configuration
    for (const auto &start_config : starting_configs) {
        JointAngles current_angles = start_config;

        // Clamp starting angles to joint limits
        current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
        current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
        current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);

        float previous_error = std::numeric_limits<float>::max();
        float stagnation_threshold = 0.001f; // If error doesn't improve by this much, consider stagnant
        int stagnation_count = 0;

        for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
            // Calculate current end-effector position using forward kinematics
            Point3D current_pos = forwardKinematics(leg, current_angles);

            // Calculate position error (delta)
            Eigen::Vector3f position_delta;
            position_delta << (p_target.x - current_pos.x),
                (p_target.y - current_pos.y),
                (p_target.z - current_pos.z);

            // Check for convergence
            float error_norm = position_delta.norm();
            if (error_norm < IK_TOLERANCE) {
                // Found good solution, record it and break from both loops
                best_result = current_angles;
                best_error = error_norm;
                goto solution_found;
            }

            // Check for stagnation (not making progress)
            if (previous_error - error_norm < stagnation_threshold) {
                stagnation_count++;
                if (stagnation_count > 5) {
                    // Algorithm is stagnating, break from inner loop to try next start
                    break;
                }
            } else {
                stagnation_count = 0; // Reset if making progress
            }
            previous_error = error_norm;

            // Calculate Jacobian matrix using DH parameters
            Eigen::Matrix3f jacobian = calculateJacobian(leg, current_angles, p_target);

            // Check for singular configuration (determinant near zero)
            float det = jacobian.determinant();
            if (std::abs(det) < 1e-6) {
                // Near singular configuration, increase damping
                const float high_damping = 0.1f;
                Eigen::Matrix3f JJT = jacobian * jacobian.transpose();
                Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
                Eigen::Matrix3f damped_inv = (JJT + high_damping * high_damping * identity).inverse();
                Eigen::Matrix3f jacobian_inverse = jacobian.transpose() * damped_inv;

                // Calculate joint angle changes with smaller step
                Eigen::Vector3f angle_delta = jacobian_inverse * position_delta * 0.5f;

                // Convert to degrees and update joint angles
                current_angles.coxa += angle_delta(0) * 180.0f / M_PI;
                current_angles.femur += angle_delta(1) * 180.0f / M_PI;
                current_angles.tibia += angle_delta(2) * 180.0f / M_PI;

                // Normalize angles to handle wraparound
                current_angles.coxa = normalizeAngle(current_angles.coxa);
                current_angles.femur = normalizeAngle(current_angles.femur);
                current_angles.tibia = normalizeAngle(current_angles.tibia);
            } else {
                // Apply standard Damped Least Squares method
                // J_inv = J^T * (J * J^T + λ^2 * I)^(-1)
                Eigen::Matrix3f JJT = jacobian * jacobian.transpose();
                Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
                Eigen::Matrix3f damped_inv = (JJT + DLS_COEFFICIENT * DLS_COEFFICIENT * identity).inverse();
                Eigen::Matrix3f jacobian_inverse = jacobian.transpose() * damped_inv;

                // Calculate joint angle changes
                Eigen::Vector3f angle_delta = jacobian_inverse * position_delta;

                // Convert to degrees and update joint angles
                current_angles.coxa += angle_delta(0) * 180.0f / M_PI;
                current_angles.femur += angle_delta(1) * 180.0f / M_PI;
                current_angles.tibia += angle_delta(2) * 180.0f / M_PI;

                // Normalize angles to handle wraparound
                current_angles.coxa = normalizeAngle(current_angles.coxa);
                current_angles.femur = normalizeAngle(current_angles.femur);
                current_angles.tibia = normalizeAngle(current_angles.tibia);
            }

            // Apply joint limits
            if (params.ik.clamp_joints) {
                current_angles.coxa = constrainAngle(current_angles.coxa, params.coxa_angle_limits[0], params.coxa_angle_limits[1]);
                current_angles.femur = constrainAngle(current_angles.femur, params.femur_angle_limits[0], params.femur_angle_limits[1]);
                current_angles.tibia = constrainAngle(current_angles.tibia, params.tibia_angle_limits[0], params.tibia_angle_limits[1]);
            }
        }

        // Check if this attempt gave a better result than previous ones
        Point3D final_pos = forwardKinematics(leg, current_angles);
        float final_error = sqrt(pow(p_target.x - final_pos.x, 2) +
                                 pow(p_target.y - final_pos.y, 2) +
                                 pow(p_target.z - final_pos.z, 2));

        // Prefer solutions with better accuracy, but also consider if solution is within joint limits
        bool current_within_limits = checkJointLimits(leg, current_angles);
        bool best_within_limits = checkJointLimits(leg, best_result);

        bool update_best = false;
        if (current_within_limits && !best_within_limits) {
            // Current solution is within limits, best is not - prefer current
            update_best = true;
        } else if (current_within_limits == best_within_limits) {
            // Both have same limit status - prefer better accuracy
            update_best = (final_error < best_error);
        }
        // If current is outside limits but best is within limits, don't update

        if (update_best) {
            best_result = current_angles;
            best_error = final_error;
        }
    }

solution_found:
    return best_result;
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
    const float base_angle_deg = leg_index * 60.0f;
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
    T *= math_utils::dhTransform(params.tibia_length, 0.0f, 0.0f, 0.0f);
    return T;
}

Eigen::Matrix3f RobotModel::calculateJacobian(int leg, const JointAngles &q, const Point3D &target) const {
    // Calculate Jacobian from DH matrices along kinematic chain
    // Based on syropod_highlevel_controller implementation

    // Get base transform for this leg
    const float base_angle_deg = leg * 60.0f;
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

    // Add final transform to tip
    Eigen::Matrix4f T_final = transforms[DOF_PER_LEG] * math_utils::dhTransform(params.tibia_length, 0.0f, 0.0f, 0.0f);

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
    return angle_rad * 180.0f / M_PI;
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
    const int resolution = 10; // discretization resolution

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
