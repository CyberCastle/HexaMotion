#include "terrain_adaptation.h"
#include "math_utils.h"
#include <algorithm>
#include <cmath>

TerrainAdaptation::TerrainAdaptation(RobotModel &model)
    : model_(model), rough_terrain_mode_(false), force_normal_touchdown_(false),
      gravity_aligned_tips_(false), touchdown_threshold_(10.0f), liftoff_threshold_(5.0f),
      step_depth_(20.0f), gravity_estimate_(0, 0, -9.81f) {

    // Initialize per-leg data
    for (int i = 0; i < NUM_LEGS; i++) {
        touchdown_detection_[i] = false;
    }
}

void TerrainAdaptation::initialize() {
    // Initialize walk plane to horizontal
    current_walk_plane_.coeffs = Eigen::Vector3f(0, 0, 0);
    current_walk_plane_.normal = Eigen::Vector3f(0, 0, 1);
    current_walk_plane_.valid = true;
    current_walk_plane_.confidence = 1.0f;

    // Clear contact history
    foot_contact_history_.clear();
}

void TerrainAdaptation::update(IFSRInterface *fsr_interface, IIMUInterface *imu_interface) {
    if (!fsr_interface || !imu_interface)
        return;

    // Update gravity estimation from IMU
    IMUData imu_data = imu_interface->readIMU();
    if (imu_data.is_valid) {
        updateGravityEstimation(imu_data);
    }

    // Update per-leg terrain detection
    for (int leg_index = 0; leg_index < NUM_LEGS; leg_index++) {
        FSRData fsr_data = fsr_interface->readFSR(leg_index);
        // FSRData doesn't have is_valid field, assume it's always valid
        detectTouchdownEvents(leg_index, fsr_data);
        updateStepPlaneDetection(leg_index, fsr_data);
    }

    // Update walk plane estimation if sufficient data available
    if (hasValidFootContactData()) {
        updateWalkPlaneEstimation();
    }
}

void TerrainAdaptation::setExternalTarget(int leg_index, const ExternalTarget &target) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        external_targets_[leg_index] = target;
    }
}

void TerrainAdaptation::setExternalDefault(int leg_index, const ExternalTarget &default_pos) {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        external_defaults_[leg_index] = default_pos;
    }
}

const TerrainAdaptation::ExternalTarget &TerrainAdaptation::getExternalTarget(int leg_index) const {
    static const ExternalTarget empty_target;
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return external_targets_[leg_index];
    }
    return empty_target;
}

const TerrainAdaptation::StepPlane &TerrainAdaptation::getStepPlane(int leg_index) const {
    static const StepPlane empty_plane;
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return step_planes_[leg_index];
    }
    return empty_plane;
}

bool TerrainAdaptation::hasTouchdownDetection(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return touchdown_detection_[leg_index];
    }
    return false;
}

Point3D TerrainAdaptation::adaptTrajectoryForTerrain(int leg_index, const Point3D &base_trajectory,
                                                     LegState leg_state, float swing_progress) {
    if (!rough_terrain_mode_ || leg_index < 0 || leg_index >= NUM_LEGS) {
        return base_trajectory;
    }

    Point3D adapted_trajectory = base_trajectory;

    // Apply external target if defined
    if (external_targets_[leg_index].defined && leg_state == SWING_PHASE) {
        // Use external target position for swing trajectory
        const ExternalTarget &target = external_targets_[leg_index];

        // Interpolate towards external target
        float blend_factor = swing_progress;
        adapted_trajectory.x = base_trajectory.x * (1.0f - blend_factor) + target.position.x * blend_factor;
        adapted_trajectory.y = base_trajectory.y * (1.0f - blend_factor) + target.position.y * blend_factor;
        adapted_trajectory.z = base_trajectory.z * (1.0f - blend_factor) + target.position.z * blend_factor;

        // Apply swing clearance
        if (swing_progress > 0.2f && swing_progress < 0.8f) {
            adapted_trajectory.z += target.swing_clearance * sin(M_PI * (swing_progress - 0.2f) / 0.6f);
        }

        return adapted_trajectory;
    }

    // Apply proactive adaptation if step plane detected
    if (step_planes_[leg_index].valid && step_planes_[leg_index].confidence > 0.5f) {
        adapted_trajectory = applyProactiveAdaptation(leg_index, adapted_trajectory);
    }
    // Apply reactive adaptation as fallback
    else if (touchdown_detection_[leg_index] && leg_state == SWING_PHASE) {
        adapted_trajectory = applyReactiveAdaptation(leg_index, adapted_trajectory);
    }

    // Force normal touchdown if enabled and in late swing phase
    if (force_normal_touchdown_ && leg_state == SWING_PHASE && swing_progress > 0.7f) {
        adapted_trajectory = forceNormalTouchdown(leg_index, adapted_trajectory);
    }

    return adapted_trajectory;
}

bool TerrainAdaptation::isTargetReachableOnTerrain(int leg_index, const Point3D &target) {
    // First check basic reachability
    JointAngles angles = model_.inverseKinematics(leg_index, target);
    Point3D fk_check = model_.forwardKinematics(leg_index, angles);

    float error = sqrt(pow(target.x - fk_check.x, 2) +
                       pow(target.y - fk_check.y, 2) +
                       pow(target.z - fk_check.z, 2));

    if (error > 10.0f) { // 10mm tolerance
        return false;
    }

    // Additional terrain-specific checks
    if (current_walk_plane_.valid) {
        // Check if target is reasonable relative to walk plane
        Point3D projected = projectOntoWalkPlane(target);
        float walk_plane_deviation = abs(target.z - projected.z);

        // Allow deviation up to step height + some margin
        if (walk_plane_deviation > 100.0f) { // 100mm max deviation
            return false;
        }
    }

    return true;
}

void TerrainAdaptation::updateWalkPlaneEstimation() {
    if (foot_contact_history_.size() < 3) {
        return; // Need at least 3 points for plane fitting
    }

    // Use least squares fitting equivalent to OpenSHC implementation
    // Plane equation: ax + by + c = z
    size_t n = foot_contact_history_.size();

    // Build matrices for least squares: A * x = B
    Eigen::MatrixXf A(n, 3);
    Eigen::VectorXf B(n);

    for (size_t i = 0; i < n; i++) {
        const Point3D &point = foot_contact_history_[i];
        A(i, 0) = point.x;
        A(i, 1) = point.y;
        A(i, 2) = 1.0f;
        B(i) = point.z;
    }

    // Solve using pseudo-inverse: x = (A^T * A)^-1 * A^T * B
    Eigen::Matrix3f AtA = A.transpose() * A;
    if (AtA.determinant() > 1e-6) { // Check for numerical stability
        Eigen::Vector3f coeffs = AtA.inverse() * A.transpose() * B;

        current_walk_plane_.coeffs = coeffs;
        current_walk_plane_.normal = Eigen::Vector3f(-coeffs[0], -coeffs[1], 1.0f).normalized();
        current_walk_plane_.valid = true;
        current_walk_plane_.confidence = std::min(1.0f, static_cast<float>(n) / 6.0f); // Max confidence with 6+ points
    }
}

// TODO: implementar método getCurrentAngles donde corresponda
void TerrainAdaptation::detectTouchdownEvents(int leg_index, const FSRData &fsr_data) {
    StepPlane &step_plane = step_planes_[leg_index];

    // Touchdown detection
    if (fsr_data.pressure > touchdown_threshold_ && !step_plane.valid) {
        // Touchdown detected - estimate step plane
        // For now, use a simple approach since we don't have getCurrentAngles
        // Get current robot pose and calculate approximate foot position
        const Parameters &params = model_.getParams();
        float base_angle = leg_index * 60.0f;
        float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
        float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));
        float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
        float safe_reach = leg_reach * 0.65f;

        Point3D foot_position;
        foot_position.x = base_x + safe_reach * cos(math_utils::degreesToRadians(base_angle));
        foot_position.y = base_y + safe_reach * sin(math_utils::degreesToRadians(base_angle));
        foot_position.z = -params.robot_height;

        step_plane.position = foot_position;
        step_plane.normal = Point3D(current_walk_plane_.normal[0],
                                    current_walk_plane_.normal[1],
                                    current_walk_plane_.normal[2]);
        step_plane.valid = true;
        step_plane.confidence = 0.8f;
        touchdown_detection_[leg_index] = true;

        // Add to contact history for walk plane estimation
        foot_contact_history_.push_back(foot_position);
        if (foot_contact_history_.size() > MAX_CONTACT_HISTORY) {
            foot_contact_history_.erase(foot_contact_history_.begin());
        }
    }
    // Liftoff detection
    else if (fsr_data.pressure < liftoff_threshold_ && step_plane.valid) {
        step_plane.valid = false;
        step_plane.confidence = 0.0f;
        touchdown_detection_[leg_index] = false;
    }
}

void TerrainAdaptation::updateStepPlaneDetection(int leg_index, const FSRData &fsr_data) {
    if (!step_planes_[leg_index].valid)
        return;

    // Update step plane confidence based on contact consistency
    StepPlane &step_plane = step_planes_[leg_index];

    if (fsr_data.in_contact) {
        // Increase confidence with stable contact
        step_plane.confidence = std::min(1.0f, step_plane.confidence + 0.1f);
    } else {
        // Decrease confidence without contact
        step_plane.confidence = std::max(0.0f, step_plane.confidence - 0.05f);
    }
}

void TerrainAdaptation::updateGravityEstimation(const IMUData &imu_data) {
    if (!imu_data.is_valid)
        return;

    // Low-pass filter for gravity estimation
    const float alpha = 0.98f; // Filter coefficient

    // Convert acceleration to gravity estimate (negate because gravity points down)
    Eigen::Vector3f accel_gravity(-imu_data.accel_x, -imu_data.accel_y, -imu_data.accel_z);

    // Apply low-pass filter
    gravity_estimate_ = alpha * gravity_estimate_ + (1.0f - alpha) * accel_gravity;

    // Normalize to expected gravity magnitude
    float magnitude = gravity_estimate_.norm();
    if (magnitude > 0.1f) {
        gravity_estimate_ = gravity_estimate_ * (9.81f / magnitude);
    }
}

Point3D TerrainAdaptation::applyProactiveAdaptation(int leg_index, const Point3D &base_trajectory) {
    const StepPlane &step_plane = step_planes_[leg_index];
    if (!step_plane.valid)
        return base_trajectory;

    Point3D adapted = base_trajectory;

    // Adjust trajectory to match detected step plane
    Point3D plane_target = step_plane.position;

    // Simple projection approach using vector calculations
    float dx = adapted.x - plane_target.x;
    float dy = adapted.y - plane_target.y;
    float dz = adapted.z - plane_target.z;

    // Project onto step plane normal (simplified)
    float dot_product = dx * step_plane.normal.x + dy * step_plane.normal.y + dz * step_plane.normal.z;

    adapted.x -= dot_product * step_plane.normal.x * step_plane.confidence;
    adapted.y -= dot_product * step_plane.normal.y * step_plane.confidence;
    adapted.z -= dot_product * step_plane.normal.z * step_plane.confidence;

    return adapted;
}

Point3D TerrainAdaptation::applyReactiveAdaptation(int leg_index, const Point3D &base_trajectory) {
    Point3D adapted = base_trajectory;

    // Apply step depth probing - extend trajectory downward
    adapted.z -= step_depth_;

    return adapted;
}

// TODO: (Simplified implementation)
Point3D TerrainAdaptation::forceNormalTouchdown(int leg_index, const Point3D &trajectory) {
    if (!current_walk_plane_.valid)
        return trajectory;

    Point3D adapted = trajectory;

    // Ensure touchdown velocity is normal to walk plane
    // This is a simplified version - full implementation would modify Bezier control points
    Point3D projected = projectOntoWalkPlane(adapted);

    // Bias towards walk plane normal approach
    float normal_bias = 0.3f; // 30% bias towards normal approach
    adapted.x = adapted.x * (1.0f - normal_bias) + projected.x * normal_bias;
    adapted.y = adapted.y * (1.0f - normal_bias) + projected.y * normal_bias;

    return adapted;
}

Point3D TerrainAdaptation::projectOntoWalkPlane(const Point3D &point) {
    if (!current_walk_plane_.valid)
        return point;

    // Project point onto walk plane: z = ax + by + c
    const Eigen::Vector3f &coeffs = current_walk_plane_.coeffs;

    Point3D projected = point;
    projected.z = coeffs[0] * point.x + coeffs[1] * point.y + coeffs[2];

    return projected;
}

bool TerrainAdaptation::hasValidFootContactData() const {
    return foot_contact_history_.size() >= 3 && current_walk_plane_.valid;
}
