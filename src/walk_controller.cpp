#include "walk_controller.h"

WalkController::WalkController(RobotModel &m)
    : model(m), current_gait(TRIPOD_GAIT), gait_phase(0.0f) {}

bool WalkController::setGaitType(GaitType gait) {
    current_gait = gait;
    gait_phase = 0.0f;
    return true;
}

bool WalkController::planGaitSequence(float, float, float) {
    return true;
}

void WalkController::updateGaitPhase(float dt) {
    gait_phase += dt;
    if (gait_phase >= 1.0f)
        gait_phase -= 1.0f;
}

Point3D WalkController::footTrajectory(int leg_index, float phase, float step_height, float step_length,
                                       float stance_duration, float swing_duration, float robot_height,
                                       const float leg_phase_offsets[NUM_LEGS], LegState leg_states[NUM_LEGS],
                                       IFSRInterface *fsr, IIMUInterface *imu) {
    float leg_phase = phase + leg_phase_offsets[leg_index];
    if (leg_phase >= 1.0f)
        leg_phase -= 1.0f;
    Point3D trajectory;
    float base_angle = leg_index * 60.0f;
    const Parameters &p = model.getParams();

    // Calculate leg base position (hexagon corner)
    float base_x = p.hexagon_radius * cos(math_utils::degreesToRadians(base_angle));
    float base_y = p.hexagon_radius * sin(math_utils::degreesToRadians(base_angle));

    // Calculate default foot position (extended leg)
    float leg_reach = p.coxa_length + p.femur_length + p.tibia_length;
    float default_foot_x = base_x + leg_reach * cos(math_utils::degreesToRadians(base_angle));
    float default_foot_y = base_y + leg_reach * sin(math_utils::degreesToRadians(base_angle));

    if (leg_phase < stance_duration) {
        leg_states[leg_index] = STANCE_PHASE;
        float support_progress = leg_phase / stance_duration;
        trajectory.x = default_foot_x + step_length * (0.5f - support_progress);
        trajectory.y = default_foot_y;
        trajectory.z = -robot_height;
    } else {
        leg_states[leg_index] = SWING_PHASE;
        float swing_progress = (leg_phase - stance_duration) / swing_duration;
        trajectory.x = default_foot_x + step_length * (swing_progress - 0.5f);
        trajectory.y = default_foot_y;
        trajectory.z = -robot_height + step_height * sin(M_PI * swing_progress);
    }
    return trajectory;
}
