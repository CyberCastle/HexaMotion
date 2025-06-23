#include "terrain_adaptation.h"
#include "hexamotion_constants.h"

/**
 * @file terrain_adaptation.cpp
 * @brief Implementation of terrain adaptation algorithms.
 */
#include "math_utils.h"
#include <algorithm>
#include <cmath>

namespace {
// File-scope empty defaults to avoid static local initialization locks
static const TerrainAdaptation::ExternalTarget EMPTY_EXTERNAL_TARGET;
static const TerrainAdaptation::StepPlane EMPTY_STEP_PLANE;
} // namespace

TerrainAdaptation::TerrainAdaptation(RobotModel &model)
    : model_(model), rough_terrain_mode_(false), force_normal_touchdown_(false),
      gravity_aligned_tips_(false), step_depth_(STEP_DEPTH_DEFAULT),
      gravity_estimate_(0, 0, -GRAVITY_ACCELERATION) {

    // Initialize FSR thresholds from model parameters or use defaults
    const Parameters &params = model_.getParams();

    // Use fsr_threshold if configured (> 0), otherwise use default
    touchdown_threshold_ = (params.fsr_threshold > 0.0f) ? params.fsr_threshold : DEFAULT_FSR_TOUCHDOWN_THRESHOLD;

    // Use fsr_liftoff_threshold if configured (> 0), otherwise use default
    liftoff_threshold_ = (params.fsr_liftoff_threshold > 0.0f) ? params.fsr_liftoff_threshold : DEFAULT_FSR_LIFTOFF_THRESHOLD;

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
    current_walk_plane_.confidence = DEFAULT_ANGULAR_SCALING;

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

        // Enhanced terrain analysis using absolute positioning data
        if (imu_data.has_absolute_capability) {
            updateAdvancedTerrainAnalysis(imu_data);
        }
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
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return external_targets_[leg_index];
    }
    return EMPTY_EXTERNAL_TARGET;
}

const TerrainAdaptation::StepPlane &TerrainAdaptation::getStepPlane(int leg_index) const {
    if (leg_index >= 0 && leg_index < NUM_LEGS) {
        return step_planes_[leg_index];
    }
    return EMPTY_STEP_PLANE;
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
        adapted_trajectory.x = base_trajectory.x * (DEFAULT_ANGULAR_SCALING - blend_factor) + target.position.x * blend_factor;
        adapted_trajectory.y = base_trajectory.y * (DEFAULT_ANGULAR_SCALING - blend_factor) + target.position.y * blend_factor;
        adapted_trajectory.z = base_trajectory.z * (DEFAULT_ANGULAR_SCALING - blend_factor) + target.position.z * blend_factor;

        // Apply swing clearance
        if (swing_progress > 0.2f && swing_progress < 0.8f) {
            adapted_trajectory.z += target.swing_clearance * sin(M_PI * (swing_progress - 0.2f) / 0.6f);
        }

        return adapted_trajectory;
    }

    // Apply proactive adaptation if step plane detected
    if (step_planes_[leg_index].valid && step_planes_[leg_index].confidence > WORKSPACE_SCALING_FACTOR) {
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
        A(i, 2) = DEFAULT_ANGULAR_SCALING;
        B(i) = point.z;
    }

    // Solve using pseudo-inverse: x = (A^T * A)^-1 * A^T * B
    Eigen::Matrix3f AtA = A.transpose() * A;
    if (AtA.determinant() > 1e-6) { // Check for numerical stability
        Eigen::Vector3f coeffs = AtA.inverse() * A.transpose() * B;

        current_walk_plane_.coeffs = coeffs;
        current_walk_plane_.normal = Eigen::Vector3f(-coeffs[0], -coeffs[1], DEFAULT_ANGULAR_SCALING).normalized();
        current_walk_plane_.valid = true;
        current_walk_plane_.confidence = std::min(DEFAULT_ANGULAR_SCALING, static_cast<float>(n) / 6.0f); // Max confidence with 6+ points
    }
}

void TerrainAdaptation::detectTouchdownEvents(int leg_index, const FSRData &fsr_data) {
    StepPlane &step_plane = step_planes_[leg_index];

    // Touchdown detection
    if (fsr_data.pressure > touchdown_threshold_ && !step_plane.valid) {
        // Touchdown detected - estimate step plane position
        // Calculate foot position using robot geometry parameters
        const Parameters &params = model_.getParams();
        float base_angle = leg_index * LEG_ANGLE_SPACING;
        float base_x = params.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
        float base_y = params.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));
        float leg_reach = params.coxa_length + params.femur_length + params.tibia_length;
        float safe_reach = leg_reach * 0.65f; // Use 65% of max reach for safety

        Point3D foot_position;
        foot_position.x = base_x + safe_reach * cos(math_utils::degreesToRadians(base_angle));
        foot_position.y = base_y + safe_reach * sin(math_utils::degreesToRadians(base_angle));
        foot_position.z = -params.robot_height;

        // Configure step plane with detected position and current walk plane normal
        step_plane.position = foot_position;
        step_plane.normal = Point3D(current_walk_plane_.normal[0],
                                    current_walk_plane_.normal[1],
                                    current_walk_plane_.normal[2]);
        step_plane.valid = true;
        step_plane.confidence = STEP_PLANE_CONFIDENCE; // High confidence for touchdown detection
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
        step_plane.confidence = std::min(DEFAULT_ANGULAR_SCALING, step_plane.confidence + MIN_SERVO_VELOCITY);
    } else {
        // Decrease confidence without contact
        step_plane.confidence = std::max(0.0f, step_plane.confidence - 0.05f);
    }
}

void TerrainAdaptation::updateGravityEstimation(const IMUData &imu_data) {
    if (!imu_data.is_valid)
        return;

    // Enhanced gravity estimation with absolute positioning support
    Eigen::Vector3f accel_gravity;
    const float alpha = 0.98f; // Filter coefficient

    // Use advanced IMU data if available (e.g., BNO055)
    if (imu_data.has_absolute_capability && imu_data.absolute_data.absolute_orientation_valid) {
        // Calculate gravity vector from absolute orientation (more accurate)
        float roll_rad = imu_data.absolute_data.absolute_roll * DEGREES_TO_RADIANS_FACTOR;
        float pitch_rad = imu_data.absolute_data.absolute_pitch * DEGREES_TO_RADIANS_FACTOR;

        // Gravity vector from absolute orientation
        accel_gravity = Eigen::Vector3f(
            sin(pitch_rad) * 9.81f,
            -sin(roll_rad) * cos(pitch_rad) * 9.81f,
            -cos(roll_rad) * cos(pitch_rad) * 9.81f);

        // Use lighter filtering for processed absolute data
        gravity_estimate_ = alpha * 1.5f * gravity_estimate_ + (DEFAULT_ANGULAR_SCALING - alpha * 1.5f) * accel_gravity;
    } else {
        // Fallback to traditional method with raw acceleration
        // Convert acceleration to gravity estimate (negate because gravity points down)
        accel_gravity = Eigen::Vector3f(-imu_data.accel_x, -imu_data.accel_y, -imu_data.accel_z);

        // Apply standard low-pass filter
        gravity_estimate_ = alpha * gravity_estimate_ + (DEFAULT_ANGULAR_SCALING - alpha) * accel_gravity;
    }

    // Normalize to expected gravity magnitude
    float magnitude = gravity_estimate_.norm();
    if (magnitude > MIN_SERVO_VELOCITY) {
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

/**
 * @brief Force normal touchdown approach for leg trajectory.
 *
 * IMPORTANT: This function implements OpenSHC-equivalent force normal touchdown.
 * OpenSHC's implementation modifies Bezier curve control nodes based on stride vectors,
 * not terrain normals. This HexaMotion implementation achieves the same functional
 * result: ensuring the leg tip approaches the landing surface at an optimal angle
 * to minimize slip and improve stability during touchdown.
 *
 * @param leg_index Index of the leg being adapted (0-5 for hexapod)
 * @param trajectory Current trajectory point for the leg tip
 * @return Point3D Adapted trajectory point optimized for touchdown approach
 */
Point3D TerrainAdaptation::forceNormalTouchdown(int leg_index, const Point3D &trajectory) {
    if (!current_walk_plane_.valid)
        return trajectory;

    // OpenSHC-equivalent force normal touchdown implementation
    // While OpenSHC modifies Bezier control nodes using stride vectors,
    // this implementation achieves the same functional goal by adjusting
    // the trajectory to create an optimal touchdown approach angle

    Point3D adapted = trajectory;

    // Get the terrain surface normal (direction of optimal approach)
    Eigen::Vector3f surface_normal = current_walk_plane_.normal;

    // Calculate optimal touchdown approach vector
    // This emulates the effect of OpenSHC's Bezier node modification
    // by creating a trajectory that approaches perpendicular to the surface
    float normal_magnitude = surface_normal.norm();

    if (normal_magnitude > 0.001f) { // Avoid division by zero
        // Normalize the surface normal
        Eigen::Vector3f unit_normal = surface_normal / normal_magnitude;

        // Calculate touchdown adjustment strength based on terrain confidence
        // Higher confidence = stronger normal approach adjustment
        float approach_strength = current_walk_plane_.confidence;

        // Apply adjustment scale appropriate for touchdown phase
        // This value is calibrated to match OpenSHC's touchdown behavior
        float adjustment_scale = 12.0f; // mm adjustment magnitude

        // Apply normal-directed adjustment to optimize touchdown approach
        // This creates the same effect as OpenSHC's Bezier modification:
        // a trajectory that approaches the surface at an optimal angle
        adapted.x = trajectory.x + unit_normal.x() * approach_strength * adjustment_scale;
        adapted.y = trajectory.y + unit_normal.y() * approach_strength * adjustment_scale;
        adapted.z = trajectory.z + unit_normal.z() * approach_strength * adjustment_scale;
    }

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

void TerrainAdaptation::updateAdvancedTerrainAnalysis(const IMUData &imu_data) {
    // Enhanced terrain analysis using absolute positioning capabilities
    if (!imu_data.absolute_data.absolute_orientation_valid)
        return;

    // Dynamic motion detection using linear acceleration (gravity-free)
    if (imu_data.absolute_data.linear_acceleration_valid) {
        Eigen::Vector3f linear_accel(
            imu_data.absolute_data.linear_accel_x,
            imu_data.absolute_data.linear_accel_y,
            imu_data.absolute_data.linear_accel_z);

        float motion_magnitude = linear_accel.norm();

        // Detect significant dynamic motion that affects terrain analysis
        if (motion_magnitude > ANGULAR_ACCELERATION_FACTOR) { // m/s² threshold for significant motion
            // Reduce confidence in terrain detection during high acceleration
            for (int i = 0; i < NUM_LEGS; i++) {
                step_planes_[i].confidence *= 0.8f;
            }
        }
    }

    // Enhanced slope detection using quaternion data
    if (imu_data.absolute_data.quaternion_valid) {
        // Use quaternion for more accurate terrain orientation
        Eigen::Vector4f quat(
            imu_data.absolute_data.quaternion_w,
            imu_data.absolute_data.quaternion_x,
            imu_data.absolute_data.quaternion_y,
            imu_data.absolute_data.quaternion_z);

        // Calculate terrain normal from quaternion (more precise than Euler angles)
        float norm = quat.norm();
        if (norm > MIN_SERVO_VELOCITY) {
            quat = quat / norm; // Normalize quaternion

            // Extract terrain normal from quaternion rotation
            Eigen::Vector3f terrain_normal(
                ANGULAR_ACCELERATION_FACTOR * (quat[1] * quat[3] - quat[0] * quat[2]),
                ANGULAR_ACCELERATION_FACTOR * (quat[2] * quat[3] + quat[0] * quat[1]),
                quat[0] * quat[0] - quat[1] * quat[1] - quat[2] * quat[2] + quat[3] * quat[3]);

            // Update terrain slope analysis
            float slope_angle = acos(abs(terrain_normal[2])) * RADIANS_TO_DEGREES_FACTOR;

            // Adjust all step planes based on quaternion-derived terrain analysis
            if (slope_angle > 15.0f) { // Significant slope detected
                for (int i = 0; i < NUM_LEGS; i++) {
                    if (step_planes_[i].valid) {
                        // Enhance normal calculation with quaternion data
                        step_planes_[i].normal.x = 0.7f * step_planes_[i].normal.x + 0.3f * terrain_normal[0];
                        step_planes_[i].normal.y = 0.7f * step_planes_[i].normal.y + 0.3f * terrain_normal[1];
                        step_planes_[i].normal.z = 0.7f * step_planes_[i].normal.z + 0.3f * terrain_normal[2];

                        // Increase confidence for quaternion-enhanced data
                        step_planes_[i].confidence = std::min(1.0f, step_planes_[i].confidence * 1.1f);
                    }
                }
            }
        }
    }

    // Calibration status consideration
    if (imu_data.absolute_data.calibration_status < 2) {
        // Reduce confidence in advanced features if IMU is not well calibrated
        for (int i = 0; i < NUM_LEGS; i++) {
            step_planes_[i].confidence *= 0.9f;
        }
    }
}

void TerrainAdaptation::updateThresholdsFromModel() {
    const Parameters &params = model_.getParams();

    // Update touchdown threshold from fsr_threshold if configured
    if (params.fsr_threshold > 0.0f) {
        touchdown_threshold_ = params.fsr_threshold;
    }

    // Update liftoff threshold from fsr_liftoff_threshold if configured
    if (params.fsr_liftoff_threshold > 0.0f) {
        liftoff_threshold_ = params.fsr_liftoff_threshold;
    }
}
