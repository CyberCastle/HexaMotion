#ifndef WALK_CONTROLLER_H
#define WALK_CONTROLLER_H

#include "model.h"

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

  private:
    RobotModel &model;
    GaitType current_gait;
    float gait_phase;
};

#endif // WALK_CONTROLLER_H
