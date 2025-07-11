#ifndef MANUAL_BODY_POSE_CONTROLLER_H
#define MANUAL_BODY_POSE_CONTROLLER_H

#include "robot_model.h"
#include <map>

/**
 * @brief Manual body posing modes equivalent to OpenSHC implementation
 *
 * Supports multiple manual body posing modes:
 * - Body translation (X, Y, Z)
 * - Body rotation (Roll, Pitch, Yaw)
 * - Individual leg manipulation
 * - Height adjustment
 * - Combined body pose control
 */
class ManualBodyPoseController {
  public:
    /**
     * @brief Available body pose control modes
     */
    enum BodyPoseMode {
        BODY_POSE_TRANSLATION,    //< Move body in X,Y,Z
        BODY_POSE_ROTATION,       //< Rotate body around X,Y,Z axes
        BODY_POSE_LEG_INDIVIDUAL, //< Control individual legs
        BODY_POSE_BODY_HEIGHT,    //< Adjust overall height
        BODY_POSE_COMBINED,       //< Combined translation and rotation
        BODY_POSE_MANUAL_BODY,    //< Manual body pose mode
        BODY_POSE_CUSTOM          //< Custom pose sequences
    };

    /**
     * @brief Body pose state structure
     */
    struct BodyPoseState {
        Point3D body_position;           //< Body position offset
        Point3D body_rotation;           //< Body rotation (roll, pitch, yaw) in radians
        Eigen::Vector4d body_quaternion; //< Body orientation as quaternion [w, x, y, z]
        Point3D leg_positions[NUM_LEGS]; //< Individual leg positions
        double body_height;               //< Body height above ground
        double pose_blend_factor;         //< Blending factor for pose transitions
        bool pose_active;                //< Whether pose is actively applied
        bool use_quaternion;             //< Whether to use quaternion or Euler angles

        BodyPoseState() : body_position(0, 0, 0), body_rotation(0, 0, 0),
                      body_quaternion(1, 0, 0, 0), // Identity quaternion
                      body_height(208.0f), pose_blend_factor(1.0f),
                      pose_active(false), use_quaternion(false) {
            for (int i = 0; i < NUM_LEGS; i++) {
                leg_positions[i] = Point3D(0, 0, 0);
            }
        }
    };

    /**
     * @brief Body pose input scaling configuration
     */
    struct BodyPoseInputScaling {
        double translation_scale; //< Translation input scaling
        double rotation_scale;    //< Rotation input scaling
        double height_scale;      //< Height input scaling
        double leg_scale;         //< Individual leg scaling

        BodyPoseInputScaling() : translation_scale(1.0f), rotation_scale(0.017453f),
                         height_scale(1.0f), leg_scale(1.0f) {}
    };

  private:
    RobotModel &model_;
    BodyPoseMode current_mode_;
    BodyPoseState current_pose_;
    BodyPoseState target_pose_;
    BodyPoseInputScaling input_scaling_;

    // Body pose limits
    struct BodyPoseLimits {
        Point3D translation_limits; // ±X, ±Y, ±Z limits
        Point3D rotation_limits;    // ±Roll, ±Pitch, ±Yaw limits
        double height_min, height_max;
        double leg_reach_limit;
    } body_pose_limits_;

    // Body pose interpolation
    double interpolation_speed_;
    bool smooth_transitions_;

    // Body pose presets
    std::map<std::string, BodyPoseState> body_pose_presets_;

  public:
    explicit ManualBodyPoseController(RobotModel &model);

    /**
     * @brief Initialize body pose controller with default settings
     */
    void initialize();

    /**
     * @brief Set active body pose control mode
     * @param mode Desired body pose mode
     */
    void setBodyPoseMode(BodyPoseMode mode);

    /**
     * @brief Get current body pose mode
     * @return Current active mode
     */
    BodyPoseMode getBodyPoseMode() const { return current_mode_; }

    /**
     * @brief Process input commands based on current mode
     * @param x X-axis input value
     * @param y Y-axis input value
     * @param z Z-axis input value
     */
    void processInput(double x, double y, double z);

    /**
     * @brief Set body pose using quaternion for orientation
     * @param position Body position offset
     * @param quaternion Body orientation as quaternion [w, x, y, z]
     * @param blend_factor Blending factor for smooth transitions
     */
    void setBodyPoseQuaternion(const Point3D &position, const Eigen::Vector4d &quaternion, double blend_factor = 1.0f);

    /**
     * @brief Interpolate to target body pose using quaternions
     * @param target_pos Target position
     * @param target_quat Target quaternion
     * @param speed Interpolation speed (0.0 to 1.0)
     */
    void interpolateToQuaternionBodyPose(const Point3D &target_pos, const Eigen::Vector4d &target_quat, double speed);

    /**
     * @brief Convert current Euler rotation to quaternion
     * @return Current orientation as quaternion
     */
    Eigen::Vector4d getCurrentQuaternion() const;

    /**
     * @brief Update pose state to use quaternions instead of Euler angles
     * @param use_quat Whether to use quaternion representation
     */
    void setUseQuaternion(bool use_quat);

    /**
     * @brief Process input with additional parameters
     * @param x X-axis input
     * @param y Y-axis input
     * @param z Z-axis input
     * @param aux Auxiliary parameter
     */
    void processInputExtended(double x, double y, double z, double aux);

    /**
     * @brief Get current body pose state
     * @return Current body pose state
     */
    const BodyPoseState &getCurrentBodyPose() const { return current_pose_; }

    /**
     * @brief Set target body pose state (for smooth transitions)
     * @param target Target body pose state
     */
    void setTargetBodyPose(const BodyPoseState &target);

    /**
     * @brief Update body pose interpolation (call at regular intervals)
     * @param dt Delta time in seconds
     */
    void updateBodyPoseInterpolation(double dt);

    /**
     * @brief Reset to neutral/default body pose
     */
    void resetBodyPose();

    /**
     * @brief Enable/disable smooth body pose transitions
     * @param enable Whether to enable smooth transitions
     * @param speed Interpolation speed (0-1)
     */
    void setSmoothBodyPoseTransitions(bool enable, double speed = 0.1f);

    /**
     * @brief Save current body pose as preset
     * @param name Preset name
     */
    void saveBodyPosePreset(const std::string &name);

    /**
     * @brief Load body pose preset
     * @param name Preset name
     * @return True if preset was found and loaded
     */
    bool loadBodyPosePreset(const std::string &name);

    /**
     * @brief Get list of available body pose presets
     * @return Vector of preset names
     */
    std::vector<std::string> getBodyPosePresetNames() const;

    /**
     * @brief Apply body pose to robot model (calculate leg positions)
     * @param pose Body pose to apply
     * @param leg_positions Output leg positions
     * @param joint_angles Output joint angles
     * @return True if body pose application succeeded
     */
    bool applyBodyPose(const BodyPoseState &pose, Point3D leg_positions[NUM_LEGS],
                   JointAngles joint_angles[NUM_LEGS]);

  private:
    void initializeBodyPoseLimits();
    void initializeDefaultBodyPosePresets();

    // Mode-specific input handlers
    void handleTranslationInput(double x, double y, double z);
    void handleRotationInput(double roll, double pitch, double yaw);
    void handleIndividualLegInput(int leg_index, double x, double y, double z);
    void handleHeightInput(double delta_height);
    void handleCombinedInput(double x, double y, double z);

    // Body pose validation and constraints
    void constrainTranslation(Point3D &translation) const;
    void constrainRotation(Point3D &rotation) const;
    void constrainHeight(double &height) const;
    void constrainLegPosition(int leg_index, Point3D &position);

    // Pose interpolation utilities
    Point3D interpolatePoint3D(const Point3D &from, const Point3D &to, double t);
    double interpolateFloat(double from, double to, double t);

    // Default pose calculation
    void calculateDefaultLegPositions();
    Point3D calculateDefaultLegPosition(int leg_index, double height);
};

#endif // MANUAL_BODY_POSE_CONTROLLER_H
