#ifndef IMU_AUTO_POSE_H
#define IMU_AUTO_POSE_H

#include "robot_model.h"
#include "manual_body_pose_controller.h"
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
        double orientation_gain;    ///< Orientation correction gain
        double inclination_gain;    ///< Inclination compensation gain
        double response_speed;      ///< Response speed (0-1)
        double deadzone_degrees;    ///< Deadzone for small tilts
        double stabilization_gain;  ///< Stabilization gain for rough terrain
        bool gravity_compensation; ///< Enable gravity compensation
        bool adaptive_gains;       ///< Use adaptive gain adjustment
        bool use_absolute_data;    ///< Use IMU's absolute positioning if available
        bool prefer_sensor_fusion; ///< Prefer sensor's built-in fusion over library algorithms

        IMUPoseParams() : orientation_gain(0.5f), inclination_gain(0.3f),
                          response_speed(0.1f), deadzone_degrees(2.0f),
                          stabilization_gain(1.0f), gravity_compensation(true),
                          adaptive_gains(false), use_absolute_data(true),
                          prefer_sensor_fusion(true) {}
    };

    /**
     * @brief Auto-pose state information
     */
    struct AutoPoseState {
        Point3D gravity_vector;     ///< Estimated gravity direction
        Point3D inclination_angle;  ///< Surface inclination (roll, pitch, yaw)
        Point3D orientation_error;  ///< Current orientation error
        Point3D correction_pose;    ///< Calculated correction pose
        bool pose_active;           ///< Whether auto-pose is active
        double confidence;           ///< Pose correction confidence (0-1)
        bool using_absolute_data;   ///< Whether using IMU's absolute positioning
        uint8_t calibration_status; ///< IMU calibration status (0-3)
        IMUMode active_mode;        ///< Current IMU operation mode

        AutoPoseState() : gravity_vector(0, 0, -9.81f), inclination_angle(0, 0, 0),
                          orientation_error(0, 0, 0), correction_pose(0, 0, 0),
                          pose_active(false), confidence(0.0f), using_absolute_data(false),
                          calibration_status(0), active_mode(IMU_MODE_RAW_DATA) {}
    };

  private:
    RobotModel &model_;
    IIMUInterface *imu_;
    ManualBodyPoseController &body_pose_controller_;
    ComputeConfig config_;

    AutoPoseMode current_mode_;
    IMUPoseParams params_;
    AutoPoseState current_state_;

    // IMU filtering and processing
    Point3D gravity_filter_;
    Point3D orientation_filter_;
    double filter_alpha_;

    // Auto-pose timing
    unsigned long last_update_time_;
    double update_interval_;

    // Adaptive control
    double terrain_roughness_estimate_;
    bool walking_detected_;
    Point3D previous_acceleration_; ///< Previous acceleration for variance calculation

  public:
    /**
     * @brief Construct IMU auto-pose controller
     * @param model Reference to robot model
     * @param imu IMU interface
     * @param body_pose_controller Reference to manual body pose controller
     * @param config Computational configuration
     */
    IMUAutoPose(RobotModel &model, IIMUInterface *imu, ManualBodyPoseController &body_pose_controller,
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
    void update(double dt);

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

    /**
     * @brief Configure IMU operation mode and preferences
     * @param use_absolute_data Whether to use absolute positioning if available
     * @param prefer_fusion Whether to prefer sensor fusion over raw data
     * @return True if configuration was successful
     */
    bool configureIMUMode(bool use_absolute_data, bool prefer_fusion);

    /**
     * @brief Get IMU calibration status
     * @return Calibration status (0-3, 3=fully calibrated)
     */
    uint8_t getIMUCalibrationStatus() const;

    /**
     * @brief Check if IMU is using absolute positioning
     * @return True if using absolute positioning data
     */
    bool isUsingAbsoluteData() const;

    /**
     * @brief Get current IMU operation mode
     * @return Current IMU mode
     */
    IMUMode getIMUMode() const;

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

    // IMU mode management
    void initializeIMUMode();
    void updateWithAbsoluteData(const IMUData &imu_data);
    void updateWithRawData(const IMUData &imu_data);
    void updateCalibrationStatus(const IMUData &imu_data);

    // Filtering and processing
    Point3D lowPassFilter(const Point3D &input, const Point3D &previous, double alpha);
    double calculateConfidence(const Point3D &orientation_error) const;
    void updateAdaptiveGains();

    // Utility functions
    Point3D normalizeAngles(const Point3D &angles);
    bool withinDeadzone(const Point3D &error) const;
};

#endif // IMU_AUTO_POSE_H
