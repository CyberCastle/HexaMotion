#ifndef IMU_AUTO_POSE_H
#define IMU_AUTO_POSE_H

#include "HexaModel.h"
#include "manual_pose_controller.h"
#include "precision_config.h"

/**
 * @brief IMU-integrated auto-posing system equivalent to OpenSHC
 *
 * Integrates IMU feedback for automatic body pose control including:
 * - Gravity-aligned body orientation
 * - Inclination compensation
 * - Dynamic balance adjustment
 * - Configurable response characteristics
 */
class IMUAutoPose {
  public:
    /**
     * @brief Auto-pose control modes
     */
    enum AutoPoseMode {
        AUTO_POSE_OFF,         ///< No automatic posing
        AUTO_POSE_LEVEL,       ///< Keep body level (gravity aligned)
        AUTO_POSE_INCLINATION, ///< Compensate for surface inclination
        AUTO_POSE_ADAPTIVE,    ///< Adaptive response based on conditions
        AUTO_POSE_CUSTOM       ///< Custom auto-pose behavior
    };

    /**
     * @brief IMU pose control parameters
     */
    struct IMUPoseParams {
        float orientation_gain;    ///< Orientation correction gain
        float inclination_gain;    ///< Inclination compensation gain
        float response_speed;      ///< Response speed (0-1)
        float deadzone_degrees;    ///< Deadzone for small tilts
        bool gravity_compensation; ///< Enable gravity compensation
        bool adaptive_gains;       ///< Use adaptive gain adjustment

        IMUPoseParams() : orientation_gain(0.5f), inclination_gain(0.3f),
                          response_speed(0.1f), deadzone_degrees(2.0f),
                          gravity_compensation(true), adaptive_gains(false) {}
    };

    /**
     * @brief Auto-pose state information
     */
    struct AutoPoseState {
        Point3D gravity_vector;    ///< Estimated gravity direction
        Point3D inclination_angle; ///< Surface inclination (roll, pitch, yaw)
        Point3D orientation_error; ///< Current orientation error
        Point3D correction_pose;   ///< Calculated correction pose
        bool pose_active;          ///< Whether auto-pose is active
        float confidence;          ///< Pose correction confidence (0-1)

        AutoPoseState() : gravity_vector(0, 0, -9.81f), inclination_angle(0, 0, 0),
                          orientation_error(0, 0, 0), correction_pose(0, 0, 0),
                          pose_active(false), confidence(0.0f) {}
    };

  private:
    RobotModel &model_;
    IIMUInterface *imu_;
    ManualPoseController &pose_controller_;
    ComputeConfig config_;

    AutoPoseMode current_mode_;
    IMUPoseParams params_;
    AutoPoseState current_state_;

    // IMU filtering and processing
    Point3D gravity_filter_;
    Point3D orientation_filter_;
    float filter_alpha_;

    // Auto-pose timing
    unsigned long last_update_time_;
    float update_interval_;

    // Adaptive control
    float terrain_roughness_estimate_;
    bool walking_detected_;

  public:
    /**
     * @brief Construct IMU auto-pose controller
     * @param model Reference to robot model
     * @param imu IMU interface
     * @param pose_controller Reference to manual pose controller
     * @param config Computational configuration
     */
    IMUAutoPose(RobotModel &model, IIMUInterface *imu, ManualPoseController &pose_controller,
                ComputeConfig config = ComputeConfig::medium());

    /**
     * @brief Initialize auto-pose system
     */
    void initialize();

    /**
     * @brief Set auto-pose mode
     * @param mode Desired auto-pose mode
     */
    void setAutoPoseMode(AutoPoseMode mode);

    /**
     * @brief Get current auto-pose mode
     * @return Current mode
     */
    AutoPoseMode getAutoPoseMode() const { return current_mode_; }

    /**
     * @brief Set IMU pose control parameters
     * @param params New parameters
     */
    void setIMUPoseParams(const IMUPoseParams &params);

    /**
     * @brief Update auto-pose control (call regularly)
     * @param dt Delta time since last update
     */
    void update(float dt);

    /**
     * @brief Get current auto-pose state
     * @return Current state information
     */
    const AutoPoseState &getAutoPoseState() const { return current_state_; }

    /**
     * @brief Enable/disable auto-pose system
     * @param enabled Whether auto-pose should be active
     */
    void setEnabled(bool enabled);

    /**
     * @brief Check if auto-pose is currently active
     * @return True if auto-pose is active
     */
    bool isActive() const { return current_state_.pose_active; }

    /**
     * @brief Set walking detection state (affects auto-pose behavior)
     * @param walking Whether robot is currently walking
     */
    void setWalkingState(bool walking);

    /**
     * @brief Get estimated gravity vector
     * @return Gravity vector in robot frame
     */
    Point3D getGravityVector() const { return current_state_.gravity_vector; }

    /**
     * @brief Get current inclination estimate
     * @return Inclination angles (roll, pitch, yaw)
     */
    Point3D getInclinationAngles() const { return current_state_.inclination_angle; }

    /**
     * @brief Reset auto-pose filters and state
     */
    void resetFilters();

  private:
    void updateIMUData();
    void updateGravityEstimate(const IMUData &imu_data);
    void updateInclinationEstimate();
    void calculateOrientationError();
    void calculateCorrectionPose();
    void applyAutoPose();

    // Mode-specific behaviors
    void updateLevelMode();
    void updateInclinationMode();
    void updateAdaptiveMode();

    // Filtering and processing
    Point3D lowPassFilter(const Point3D &input, const Point3D &previous, float alpha);
    float calculateConfidence(const Point3D &orientation_error);
    void updateAdaptiveGains();

    // Utility functions
    Point3D quaternionToEuler(float qw, float qx, float qy, float qz);
    Point3D normalizeAngles(const Point3D &angles);
    bool withinDeadzone(const Point3D &error);
};

#endif // IMU_AUTO_POSE_H
