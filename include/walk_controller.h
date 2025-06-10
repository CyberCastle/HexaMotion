#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "model.h"
#include "terrain_adaptation.h"
#include "velocity_limits.h"

class WalkController {
  public:
    explicit WalkController(RobotModel &model);

    bool setGaitType(GaitType gait);
    bool planGaitSequence(float vx, float vy, float omega);
    void updateGaitPhase(float dt);

    // Velocity limiting methods
    VelocityLimits::LimitValues getVelocityLimits(float bearing_degrees = 0.0f) const;
    VelocityLimits::LimitValues applyVelocityLimits(float vx, float vy, float omega) const;
    bool validateVelocityCommand(float vx, float vy, float omega) const;
    void updateVelocityLimits(float frequency, float stance_ratio, float time_to_max_stride = 2.0f);

    // Velocity limiting configuration
    void setVelocitySafetyMargin(float margin);
    void setAngularVelocityScaling(float scaling);
    VelocityLimits::WorkspaceConfig getWorkspaceConfig() const;
    Point3D footTrajectory(int leg, float phase, float step_height, float step_length,
                           float stance_duration, float swing_duration, float robot_height,
                           const float leg_phase_offsets[NUM_LEGS], LegState leg_states[NUM_LEGS],
                           IFSRInterface *fsr, IIMUInterface *imu);

    // Terrain adaptation methods
    void enableRoughTerrainMode(bool enabled);
    void enableForceNormalTouchdown(bool enabled);
    void enableGravityAlignedTips(bool enabled);
    void setExternalTarget(int leg_index, const TerrainAdaptation::ExternalTarget &target);
    void setExternalDefault(int leg_index, const TerrainAdaptation::ExternalTarget &default_pos);

    // Terrain state accessors
    const TerrainAdaptation::WalkPlane &getWalkPlane() const;
    const TerrainAdaptation::ExternalTarget &getExternalTarget(int leg_index) const;
    const TerrainAdaptation::StepPlane &getStepPlane(int leg_index) const;
    bool hasTouchdownDetection(int leg_index) const;
    Eigen::Vector3f estimateGravity() const;

  private:
    RobotModel &model;
    GaitType current_gait;
    float gait_phase;

    // Terrain adaptation system
    TerrainAdaptation terrain_adaptation_;

    // Velocity limits system
    VelocityLimits velocity_limits_;
    VelocityLimits::LimitValues current_velocity_limits_;
    VelocityLimits::LimitValues current_velocities_;
};

#endif // WALK_CONTROLLER_H
