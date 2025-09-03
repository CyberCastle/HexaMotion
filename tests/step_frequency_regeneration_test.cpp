#include "gait_config_factory.h"
#include "robot_model.h"
#include <cassert>
#include <cmath>
#include <iostream>

// Simple helper to build Parameters with minimal required fields.
static Parameters makeParams(double step_freq) {
    Parameters p{}; // value-initialize
    p.coxa_length = 50;
    p.femur_length = 80;
    p.tibia_length = 120;
    p.hexagon_radius = 120;
    p.robot_height = 200;
    p.robot_weight = 1.0;
    p.center_of_mass = Eigen::Vector3d(0, 0, 0);
    p.coxa_angle_limits[0] = -45;
    p.coxa_angle_limits[1] = 45;
    p.femur_angle_limits[0] = -60;
    p.femur_angle_limits[1] = 60;
    p.tibia_angle_limits[0] = -75;
    p.tibia_angle_limits[1] = 75;
    p.step_frequency = step_freq; // target frequency under test
    p.time_delta = 0.02;          // 50 Hz loop
    return p;
}

static void validateCycle(const GaitConfiguration &cfg, const char *label) {
    StepCycle sc = cfg.generateStepCycle();
    int base = cfg.phase_config.stance_phase + cfg.phase_config.swing_phase;
    assert(base > 0);
    // Period must be a positive multiple of base
    assert(sc.period_ % base == 0);
    int normaliser = sc.period_ / base;
    assert(normaliser >= 1);
    // Stance/swing partitions
    assert(sc.stance_period_ + sc.swing_period_ == sc.period_);
    assert(sc.stance_period_ == cfg.phase_config.stance_phase * normaliser);
    assert(sc.swing_period_ == cfg.phase_config.swing_phase * normaliser);
    // Frequency recomputed from period/time_delta must match stored frequency_ (within tolerance)
    double recomputed_freq = 1.0 / (sc.period_ * cfg.time_delta);
    double rel_err = std::fabs(recomputed_freq - sc.frequency_) / std::max(1e-9, sc.frequency_);
    assert(rel_err < 1e-9);
    // Effective realised frequency (after normalization) should be reasonably close (<= 15%) to configured step_frequency
    double cfg_freq = cfg.step_frequency;
    double realised_err = std::fabs(recomputed_freq - cfg_freq) / std::max(1e-9, cfg_freq);
    assert(realised_err <= 0.15);
    std::cout << label << ": configured=" << cfg_freq << "Hz realised=" << recomputed_freq
              << "Hz period_iters=" << sc.period_ << " normaliser=" << normaliser << "\n";
}

int main() {
    // Test a few frequencies, including one that forces a higher normaliser
    double freqs[] = {0.5, 1.0, 1.7, 2.0};
    for (double f : freqs) {
        Parameters p = makeParams(f);
        // Build tripod gait (simple balanced gait)
        GaitConfiguration tripod = createTripodGaitConfig(p);
        // Ensure time_delta propagated
        assert(std::fabs(tripod.time_delta - p.time_delta) < 1e-12);
        validateCycle(tripod, "tripod");
    }

    // Also verify wave gait (different phase ratios) at one frequency
    Parameters p_wave = makeParams(1.25);
    GaitConfiguration wave = createWaveGaitConfig(p_wave);
    assert(std::fabs(wave.time_delta - p_wave.time_delta) < 1e-12);
    validateCycle(wave, "wave");

    std::cout << "Step frequency regeneration test PASSED" << std::endl;
    return 0;
}
