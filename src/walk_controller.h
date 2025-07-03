#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "HexaModel.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"
#include "workspace_validator.h"
#include <memory>

/**
 * @brief High level walking controller handling gaits and terrain adaptation.
 */
class WalkController {
  public:
    /**
     * @brief Construct a walk controller.
     * @param model Reference to the robot model used for kinematics.
     */
    explicit WalkController(RobotModel &model);

    /**
     * @brief Select the active gait type.
     * @param gait Desired gait enumeration value.
     * @return True if the gait was set successfully.
     */
    bool setGaitType(GaitType gait);

    /**
     * @brief Plan the next gait sequence given velocity commands.
     * @param vx Linear velocity in X (m/s).
     * @param vy Linear velocity in Y (m/s).
     * @param omega Angular velocity around Z (rad/s).
     * @return True if planning succeeded.
     */
    bool planGaitSequence(double vx, double vy, double omega);

    /**
     * @brief Update internal gait phase progression.
     * @param dt Time step in seconds.
     */
    void updateGaitPhase(double dt);

    // Gait phase management
    double getGaitPhase() const { return gait_phase; }

    // Velocity limiting methods
    /**
     * @brief Get velocity limits for a given bearing.
     * @param bearing_degrees Direction of travel in degrees.
     * @return Calculated limit values.
     */
    VelocityLimits::LimitValues getVelocityLimits(double bearing_degrees = 0.0f) const;

    /**
     * @brief Apply velocity limits to a commanded velocity vector.
     * @param vx Desired X velocity.
     * @param vy Desired Y velocity.
     * @param omega Desired angular velocity.
     * @return Limited velocity values.
     */
    VelocityLimits::LimitValues applyVelocityLimits(double vx, double vy, double omega) const;

    /**
     * @brief Validate a velocity command against computed limits.
     * @param vx Commanded X velocity.
     * @param vy Commanded Y velocity.
     * @param omega Commanded angular velocity.
     * @return True if the command is within limits.
     */
    bool validateVelocityCommand(double vx, double vy, double omega) const;

    /**
     * @brief Update velocity limit tables for new gait parameters.
     * @param frequency Step frequency in Hz.
     * @param stance_ratio Ratio of stance phase (0-1).
     * @param time_to_max_stride Time to reach maximum stride length.
     */
    void updateVelocityLimits(double frequency, double stance_ratio, double time_to_max_stride = 2.0f);

    // Velocity limiting configuration
    /** Set workspace safety margin used for velocity limiting. */
    void setVelocitySafetyMargin(double margin);
    /** Set scaling factor for angular velocity commands. */
    void setAngularVelocityScaling(double scaling);
    /** Retrieve the current workspace configuration. */
    VelocityLimits::WorkspaceConfig getWorkspaceConfig() const;

    /**
     * @brief Calculate foot trajectory for a single leg.
     * @param leg Index of the leg (0-5).
     * @param phase Current gait phase (0-1).
     * @param step_height Step height in mm.
     * @param step_length Step length in mm.
     * @param stance_duration Stance phase duration fraction.
     * @param swing_duration Swing phase duration fraction.
     * @param robot_height Current body height.
     * @param leg_phase_offsets Phase offset array for each leg.
     * @param leg_states Array of leg state values.
     * @param fsr Interface to FSR sensors.
     * @param imu Interface to IMU.
     * @return Calculated foot position in world frame.
     */
    Point3D footTrajectory(int leg, double phase, double step_height, double step_length,
                           double stance_duration, double swing_duration, double robot_height,
                           const double leg_phase_offsets[NUM_LEGS], LegState leg_states[NUM_LEGS],
                           IFSRInterface *fsr, IIMUInterface *imu);

    // Terrain adaptation methods
    /**
     * @brief Enable rough terrain mode with advanced features
     * @param enabled Whether to enable rough terrain mode
     * @param force_normal_touchdown Force touchdown normal to terrain
     * @param proactive_adaptation Use proactive terrain adaptation
     */
    void enableRoughTerrainMode(bool enabled, bool force_normal_touchdown = true,
                                bool proactive_adaptation = true);

    /**
     * @brief Enable force normal touchdown mode
     * @param enabled Whether to force normal touchdown to walk plane
     */
    void enableForceNormalTouchdown(bool enabled);

    /**
     * @brief Enable gravity-aligned tips mode
     * @param enabled Whether tips should align with gravity
     */
    void enableGravityAlignedTips(bool enabled);

    /** Set an external target for a specific leg. */
    void setExternalTarget(int leg_index, const TerrainAdaptation::ExternalTarget &target);
    /** Set an external default position for a leg. */
    void setExternalDefault(int leg_index, const TerrainAdaptation::ExternalTarget &default_pos);

    // Terrain state accessors
    /**
     * @brief Get current walk plane estimation
     * @return Current walk plane structure
     */
    const TerrainAdaptation::WalkPlane &getWalkPlane() const;

    /** Retrieve the external target for a leg if available. */
    const TerrainAdaptation::ExternalTarget &getExternalTarget(int leg_index) const;
    /** Get the detected step plane for a specific leg. */
    const TerrainAdaptation::StepPlane &getStepPlane(int leg_index) const;
    /** Check if touchdown detection is active for a leg. */
    bool hasTouchdownDetection(int leg_index) const;

    /**
     * @brief Estimate gravity vector from IMU
     * @return Estimated gravity vector
     */
    Eigen::Vector3d estimateGravity() const;

    /**
     * @brief Get current velocity commands for gait pattern decisions
     * @return Current velocity values stored from last planGaitSequence call
     */
    const VelocityLimits::LimitValues &getCurrentVelocities() const;

  private:
    RobotModel &model;
    GaitType current_gait;
    double gait_phase;

    // Terrain adaptation system
    TerrainAdaptation terrain_adaptation_;

    // Velocity limits system
    VelocityLimits velocity_limits_;
    VelocityLimits::LimitValues current_velocity_limits_;
    VelocityLimits::LimitValues current_velocities_;

    // Workspace validation
    std::unique_ptr<WorkspaceValidator> workspace_validator_;

    // Collision avoidance: track current leg positions
    Point3D current_leg_positions_[NUM_LEGS];
};

#endif // WALK_CONTROLLER_H
