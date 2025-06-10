#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "model.h"
#include "terrain_adaptation.h"

class WalkController {
  public:
    explicit WalkController(RobotModel &model);

    bool setGaitType(GaitType gait);
    bool planGaitSequence(float vx, float vy, float omega);
    void updateGaitPhase(float dt);
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
};

#endif // WALK_CONTROLLER_H
