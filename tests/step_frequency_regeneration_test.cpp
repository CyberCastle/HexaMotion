#include "gait_config_factory.h"
#include "robot_model.h"
#include <cassert>
#include <cmath>
#include <iostream>

/** Simple helper to build Parameters with minimal required fields. */
static Parameters makeParams(double step_freq) {
    /** Value-initialize. */
    Parameters p{};
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
    /** Target frequency under test. */
    p.step_frequency = step_freq;
    /** 50 Hz loop. */
    p.time_delta = 0.02;
    return p;
}

static void validateCycle(const GaitConfiguration &cfg, const char *label) {
    StepCycle sc = cfg.generateStepCycle();
    int base = cfg.phase_config.stance_phase + cfg.phase_config.swing_phase;
    assert(base > 0);
    /** Period must be a positive multiple of base. */
    assert(sc.period_ % base == 0);
    int normaliser = sc.period_ / base;
    assert(normaliser >= 1);
    /** Stance/swing partitions. */
    assert(sc.stance_period_ + sc.swing_period_ == sc.period_);
    assert(sc.stance_period_ == cfg.phase_config.stance_phase * normaliser);
    assert(sc.swing_period_ == cfg.phase_config.swing_phase * normaliser);
    /** Frequency recomputed from period/time_delta must match stored frequency_ (within tolerance). */
    double recomputed_freq = 1.0 / (sc.period_ * cfg.time_delta);
    double rel_err = std::fabs(recomputed_freq - sc.frequency_) / std::max(1e-9, sc.frequency_);
    assert(rel_err < 1e-9);
    /** Effective realized frequency should be within 15% of configured step_frequency.
     *  OpenSHC convention: step_frequency is normalized assuming swing_ratio=1.0.
     *  The actual (effective) stepping rate is step_frequency * swing_ratio.
     *  See OpenSHC/config/readme.md "step_frequency" documentation. */
    double cfg_freq = cfg.step_frequency;
    double swing_ratio = double(cfg.phase_config.swing_phase) / double(base);
    double effective_freq = cfg_freq * swing_ratio;
    double realised_err = std::fabs(recomputed_freq - effective_freq) / std::max(1e-9, effective_freq);
    assert(realised_err <= 0.15);
    std::cout << label << ": configured=" << cfg_freq << "Hz realised=" << recomputed_freq
              << "Hz period_iters=" << sc.period_ << " normaliser=" << normaliser << "\n";
}

int main() {
    /** Test a few frequencies, including one that forces a higher normalizer. */
    double freqs[] = {0.5, 1.0, 1.7, 2.0};
    for (double f : freqs) {
        Parameters p = makeParams(f);
        /** Build tripod gait (simple balanced gait). */
        GaitConfiguration tripod = createTripodGaitConfig(p);
        /** Ensure time_delta is propagated. */
        assert(std::fabs(tripod.time_delta - p.time_delta) < 1e-12);
        validateCycle(tripod, "tripod");
    }

    /** Also verify wave gait (different phase ratios) at one frequency. */
    Parameters p_wave = makeParams(1.25);
    GaitConfiguration wave = createWaveGaitConfig(p_wave);
    assert(std::fabs(wave.time_delta - p_wave.time_delta) < 1e-12);
    validateCycle(wave, "wave");

    std::cout << "Step frequency regeneration test PASSED" << std::endl;
    return 0;
}
