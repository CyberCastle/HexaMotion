#include "admittance_controller.h"

AdmittanceController::AdmittanceController(RobotModel &m, IIMUInterface *i, IFSRInterface *f)
    : model(m), imu(i), fsr(f) {}

Eigen::Vector3f AdmittanceController::orientationError(const Eigen::Vector3f &target) {
    IMUData imu_data = imu->readIMU();
    Eigen::Vector3f current(imu_data.roll, imu_data.pitch, imu_data.yaw);
    return target - current;
}

bool AdmittanceController::maintainOrientation(const Eigen::Vector3f &target, Eigen::Vector3f &current, float dt) {
    if (!imu)
        return false;
    Eigen::Vector3f err = orientationError(target);
    current += err * dt * 0.5f;
    return true;
}

bool AdmittanceController::checkStability(const Point3D leg_pos[NUM_LEGS], const LegState leg_states[NUM_LEGS]) {
    if (!fsr)
        return true;
    int contacts = 0;
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (leg_states[i] == STANCE_PHASE) {
            FSRData fsr_data = fsr->readFSR(i);
            if (fsr_data.in_contact)
                contacts++;
        }
    }
    return contacts >= 3;
}
