#ifndef S_CURVE_PROFILE_H
#define S_CURVE_PROFILE_H

#include <array>
#include <cmath>

/**
 * @brief Jerk-limited (S-curve) 7‑segment angular motion profile.
 *
 * Generates a time‑optimal symmetric profile (accel, cruise, decel) with bounded
 * jerk, acceleration and velocity for a single angular DOF. Supports automatic
 * degeneration to profiles without constant acceleration or velocity plateaus
 * when the move distance is short.
 *
 * Segment layout (durations t1..t7):
 *   1) +J  (accel ramps 0 -> A_peak)
 *   2)  0  (accel held at +A_peak)
 *   3) -J  (accel ramps +A_peak -> 0)
 *   4)  0  (cruise, accel 0, velocity Va)
 *   5) -J  (accel ramps 0 -> -A_peak)
 *   6)  0  (accel held at -A_peak)
 *   7) +J  (accel ramps -A_peak -> 0)
 * Any of segments 2,4,6 may collapse to zero duration depending on distance / limits.
 */
class SCurveProfile {
  public:
    struct Sample {
        double position;     ///< Angular position
        double velocity;     ///< Angular velocity
        double acceleration; ///< Angular acceleration
    };

    /**
     * @param start_angle  Initial angle (rad or deg – caller consistent)
     * @param target_angle Target angle
     * @param vmax         Max velocity (>0)
     * @param amax         Max acceleration (>0)
     * @param jmax         Max jerk (>0)
     */
    SCurveProfile(double start_angle, double target_angle,
                  double vmax, double amax, double jmax) {
        init(start_angle, target_angle, vmax, amax, jmax);
    }

    SCurveProfile() = default;

    void init(double start_angle, double target_angle,
              double vmax, double amax, double jmax) {
        start_angle_ = start_angle;
        target_angle_ = target_angle;
        vmax_ = std::max(1e-12, std::abs(vmax));
        amax_input_ = std::max(1e-12, std::abs(amax));
        jmax_ = std::max(1e-12, std::abs(jmax));
        direction_ = (target_angle_ >= start_angle_) ? 1.0 : -1.0;
        displacement_ = std::abs(target_angle_ - start_angle_);
        computeProfile();
    }

    /** Total motion time. */
    double totalTime() const { return total_time_; }

    /** True if computation succeeded (distance >= 0 always true). */
    bool valid() const { return total_time_ > 0.0; }

    /**
     * @brief Sample the profile at time t.
     * @param t Time from 0..totalTime (clamped).
     * @return Sample {position, velocity, acceleration}.
     */
    Sample sample(double t) const;

  private:
    // Inputs
    double start_angle_{0.0};
    double target_angle_{0.0};
    double vmax_{0.0};
    double amax_input_{0.0};
    double jmax_{0.0};
    // Derived
    double direction_{1.0};
    double displacement_{0.0};

    // Actual peak acceleration used (may be < amax_input_ for short moves / velocity limit)
    double a_peak_{0.0};
    // Cruise velocity actually achieved (<= vmax_)
    double v_cruise_{0.0};

    // Segment durations t1..t7
    std::array<double, 7> dt_{{0, 0, 0, 0, 0, 0, 0}};
    // Cumulative times (segment start times) size 8 (last=total)
    std::array<double, 8> t_cum_{{0, 0, 0, 0, 0, 0, 0, 0}};

    // Precomputed segment start states (position, velocity, acceleration)
    struct State {
        double p;
        double v;
        double a;
    };
    std::array<State, 8> seg_state_{}; // index i = start of segment i (0..7), 7=end

    double total_time_{0.0};

    void computeProfile();
    void buildCumulative();
    void precomputeStates();
};

#endif // S_CURVE_PROFILE_H
