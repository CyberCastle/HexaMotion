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
    float reach = p.coxa_length + p.femur_length + p.tibia_length;
    float base_radius = model.getBaseRadius();
    float base_x = base_radius * cos(math_utils::degreesToRadians(base_angle));
    float base_y = base_radius * sin(math_utils::degreesToRadians(base_angle));

    // Default foot position placed to keep the leg within reach even at low
    // robot heights. Position the foot forward from the hip by the minimum
    // horizontal distance allowed by the leg geometry.
    float min_horiz = sqrtf(std::max(
        (p.tibia_length - p.femur_length) * (p.tibia_length - p.femur_length) -
            robot_height * robot_height,
        0.0f));
    float max_horiz = sqrtf(std::max(reach * reach - robot_height * robot_height, 0.0f)) - p.coxa_length;
    min_horiz = std::min(min_horiz, max_horiz);
    float default_foot_x = base_x + p.coxa_length + min_horiz;
    float default_foot_y = base_y;

    if (leg_phase < stance_duration) {
        leg_states[leg_index] = STANCE_PHASE;
        float support_progress = leg_phase / stance_duration;
        trajectory.x = default_foot_x + step_length * (0.5f - support_progress);
        float rel = trajectory.x - base_x - p.coxa_length;
        if (fabs(rel) > max_horiz)
            trajectory.x = base_x + p.coxa_length + copysignf(max_horiz, rel);
        trajectory.y = default_foot_y;
        trajectory.z = -robot_height;
    } else {
        leg_states[leg_index] = SWING_PHASE;
        float swing_progress = (leg_phase - stance_duration) / swing_duration;
        trajectory.x = default_foot_x + step_length * (swing_progress - 0.5f);
        float rel2 = trajectory.x - base_x - p.coxa_length;
        if (fabs(rel2) > max_horiz)
            trajectory.x = base_x + p.coxa_length + copysignf(max_horiz, rel2);
        trajectory.y = default_foot_y;
        trajectory.z = -robot_height + step_height * sin(M_PI * swing_progress);
    }
    return trajectory;
}
