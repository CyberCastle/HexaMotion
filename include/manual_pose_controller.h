#ifndef MANUAL_POSE_CONTROLLER_H
#define MANUAL_POSE_CONTROLLER_H

#include "HexaModel.h"
#include <map>

/**
 * @brief Manual posing modes equivalent to OpenSHC implementation
 *
 * Supports multiple manual posing modes:
 * - Body translation (X, Y, Z)
 * - Body rotation (Roll, Pitch, Yaw)
 * - Individual leg manipulation
 * - Height adjustment
 * - Combined pose control
 */
class ManualPoseController {
  public:
    /**
     * @brief Available pose control modes
     */
    enum PoseMode {
        POSE_TRANSLATION,    ///< Move body in X,Y,Z
        POSE_ROTATION,       ///< Rotate body around X,Y,Z axes
        POSE_LEG_INDIVIDUAL, ///< Control individual legs
        POSE_BODY_HEIGHT,    ///< Adjust overall height
        POSE_COMBINED,       ///< Combined translation and rotation
        POSE_MANUAL_BODY,    ///< Manual body pose mode
        POSE_CUSTOM          ///< Custom pose sequences
    };

    /**
     * @brief Pose state structure
     */
    struct PoseState {
        Point3D body_position;           ///< Body position offset
        Point3D body_rotation;           ///< Body rotation (roll, pitch, yaw) in radians
        Eigen::Vector4f body_quaternion; ///< Body orientation as quaternion [w, x, y, z]
        Point3D leg_positions[NUM_LEGS]; ///< Individual leg positions
        float body_height;               ///< Body height above ground
        float pose_blend_factor;         ///< Blending factor for pose transitions
        bool pose_active;                ///< Whether pose is actively applied
        bool use_quaternion;             ///< Whether to use quaternion or Euler angles

        PoseState() : body_position(0, 0, 0), body_rotation(0, 0, 0),
                      body_quaternion(1, 0, 0, 0), // Identity quaternion
                      body_height(100.0f), pose_blend_factor(1.0f),
                      pose_active(false), use_quaternion(false) {
            for (int i = 0; i < NUM_LEGS; i++) {
                leg_positions[i] = Point3D(0, 0, 0);
            }
        }
    };

    /**
     * @brief Input scaling configuration
     */
    struct InputScaling {
        float translation_scale; ///< Translation input scaling
        float rotation_scale;    ///< Rotation input scaling
        float height_scale;      ///< Height input scaling
        float leg_scale;         ///< Individual leg scaling

        InputScaling() : translation_scale(1.0f), rotation_scale(0.017453f),
                         height_scale(1.0f), leg_scale(1.0f) {}
    };

  private:
    RobotModel &model_;
    PoseMode current_mode_;
    PoseState current_pose_;
    PoseState target_pose_;
    InputScaling input_scaling_;

    // Pose limits
    struct PoseLimits {
        Point3D translation_limits; // ±X, ±Y, ±Z limits
        Point3D rotation_limits;    // ±Roll, ±Pitch, ±Yaw limits
        float height_min, height_max;
        float leg_reach_limit;
    } pose_limits_;

    // Pose interpolation
    float interpolation_speed_;
    bool smooth_transitions_;

    // Pose presets
    std::map<std::string, PoseState> pose_presets_;

  public:
    explicit ManualPoseController(RobotModel &model);

    /**
     * @brief Initialize pose controller with default settings
     */
    void initialize();

    /**
     * @brief Set active pose control mode
     * @param mode Desired pose mode
     */
    void setPoseMode(PoseMode mode);

    /**
     * @brief Get current pose mode
     * @return Current active mode
     */
    PoseMode getPoseMode() const { return current_mode_; }

    /**
     * @brief Process input commands based on current mode
     * @param x X-axis input value
     * @param y Y-axis input value
     * @param z Z-axis input value
     */
    void processInput(float x, float y, float z);

    /**
     * @brief Set pose using quaternion for orientation
     * @param position Body position offset
     * @param quaternion Body orientation as quaternion [w, x, y, z]
     * @param blend_factor Blending factor for smooth transitions
     */
    void setPoseQuaternion(const Point3D &position, const Eigen::Vector4f &quaternion, float blend_factor = 1.0f);

    /**
     * @brief Interpolate to target pose using quaternions
     * @param target_pos Target position
     * @param target_quat Target quaternion
     * @param speed Interpolation speed (0.0 to 1.0)
     */
    void interpolateToQuaternionPose(const Point3D &target_pos, const Eigen::Vector4f &target_quat, float speed);

    /**
     * @brief Convert current Euler rotation to quaternion
     * @return Current orientation as quaternion
     */
    Eigen::Vector4f getCurrentQuaternion() const;

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
    void processInputExtended(float x, float y, float z, float aux);

    /**
     * @brief Get current pose state
     * @return Current pose state
     */
    const PoseState &getCurrentPose() const { return current_pose_; }

    /**
     * @brief Set target pose state (for smooth transitions)
     * @param target Target pose state
     */
    void setTargetPose(const PoseState &target);

    /**
     * @brief Update pose interpolation (call at regular intervals)
     * @param dt Delta time in seconds
     */
    void updatePoseInterpolation(float dt);

    /**
     * @brief Reset to neutral/default pose
     */
    void resetPose();

    /**
     * @brief Enable/disable smooth pose transitions
     * @param enable Whether to enable smooth transitions
     * @param speed Interpolation speed (0-1)
     */
    void setSmoothTransitions(bool enable, float speed = 0.1f);

    /**
     * @brief Save current pose as preset
     * @param name Preset name
     */
    void savePosePreset(const std::string &name);

    /**
     * @brief Load pose preset
     * @param name Preset name
     * @return True if preset was found and loaded
     */
    bool loadPosePreset(const std::string &name);

    /**
     * @brief Get list of available pose presets
     * @return Vector of preset names
     */
    std::vector<std::string> getPosePresetNames() const;

    /**
     * @brief Apply pose to robot model (calculate leg positions)
     * @param pose Pose to apply
     * @param leg_positions Output leg positions
     * @param joint_angles Output joint angles
     * @return True if pose application succeeded
     */
    bool applyPose(const PoseState &pose, Point3D leg_positions[NUM_LEGS],
                   JointAngles joint_angles[NUM_LEGS]);

  private:
    void initializePoseLimits();
    void initializeDefaultPresets();

    // Mode-specific input handlers
    void handleTranslationInput(float x, float y, float z);
    void handleRotationInput(float roll, float pitch, float yaw);
    void handleIndividualLegInput(int leg_index, float x, float y, float z);
    void handleHeightInput(float delta_height);
    void handleCombinedInput(float x, float y, float z);

    // Pose validation and constraints
    void constrainTranslation(Point3D &translation);
    void constrainRotation(Point3D &rotation);
    void constrainHeight(float &height);
    void constrainLegPosition(int leg_index, Point3D &position);

    // Pose interpolation utilities
    Point3D interpolatePoint3D(const Point3D &from, const Point3D &to, float t);
    float interpolateFloat(float from, float to, float t);

    // Default pose calculation
    void calculateDefaultLegPositions();
    Point3D calculateDefaultLegPosition(int leg_index, float height);
};

#endif // MANUAL_POSE_CONTROLLER_H
