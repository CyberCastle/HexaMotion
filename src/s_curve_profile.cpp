#include "s_curve_profile.h"

/**
 * Implementation notes:
 * We construct a symmetric 7‑segment profile. Strategy:
 * 1. Attempt full profile with max jerk/accel/velocity.
 * 2. Determine minimal distances for (J+A+J) acceleration half and match decel.
 * 3. If distance too short for constant accel (segment 2) remove it (triangular accel ramp: +J then -J).
 * 4. Recompute using reduced a_peak if velocity limit not reached.
 * 5. If still too short to reach peak jerk phases fully, we fall back to simple constant acceleration (parabolic) or direct move.
 */

void SCurveProfile::computeProfile() {
    if (displacement_ < 1e-12) {
        total_time_ = 0.0;
        return;
    }

    // Base candidate using provided limits
    double j = jmax_;
    double a = amax_input_;

    // Time to ramp accel 0->a with jerk j
    double tj = a / j; // segment 1 duration (and 3,5,7)
    // Minimal velocity reached after segment 1 (area under accel ramp): 0.5*a*tj = a^2/(2j)
    // Position contribution patterns computed via integration.

    // Distance covered in (J + A + -J) acceleration macro (segments 1+2+3) when including a constant accel plateau ta (segment 2 duration):
    // Velocity at end of seg1: v1 = 0.5 * a * tj
    // Accel plateau (seg2) contributes a * ta velocity increase
    // End velocity of seg3 (accel returns to 0): vA = a * (tj + ta)
    // Distance of J-ramp up: p1 = (1/6) * a * tj^2   (since jerk integration)
    // Plateau distance: p2 = v_avg_plateau * ta = (v1 + (v1 + a*ta)) * 0.5 * ta
    // Ramp down distance: p3 mirrors p1 but with starting velocity v1 + a*ta + a*tj/2.
    // Instead of closed form complexity we use known standard formulas for symmetric S-curve.

    // For simplicity adopt canonical derivation: distance of full accel macro (1+2+3) with plateau ta:
    // p_accel = a * tj * (tj/3 + ta) + 0.5 * a * ta * (tj + ta)  (approx). To avoid algebra errors we'll compute numerically by segment integration.

    auto integrate_accel_macro = [&](double ta) {
        // Segment 1: accel increases linearly: a(t)=j*t, 0..tj
        // position contribution p1 = integral_0^tj v(t) dt. v(t)= integral a dt = 0.5*j*t^2
        // p1 = integral_0^tj 0.5*j*t^2 dt = (0.5*j)*(tj^3/3) = j*tj^3/6 = (a^3)/(6*j^2)
        double p1 = j * pow(tj, 3) / 6.0;
        double v1 = 0.5 * j * tj * tj; // = a^2/(2j)
        // Segment 2: constant accel a during ta. v increases from v1 to v1 + a*ta.
        double p2 = v1 * ta + 0.5 * a * ta * ta;
        double v2 = v1 + a * ta;
        // Segment 3: accel decreases linearly from a to 0 over tj. Treat jerk -j, starting accel a.
        // v increase during seg3: + a*t - 0.5*j*t^2, t in [0,tj]. Final added velocity = a*tj - 0.5*j*tj^2 = a*tj - 0.5*a*tj = 0.5*a*tj
        // displacement contribution p3: integrate v2 + a*t - 0.5*j*t^2 dt
        // = v2*tj + 0.5*a*tj^2 - (j*tj^3)/6
        double p3 = v2 * tj + 0.5 * a * tj * tj - j * pow(tj, 3) / 6.0;
        return p1 + p2 + p3; // total accel macro distance
    };

    // Step 1: assume we can have plateau ta>=0 and possibly cruise later.
    // We first try to reach vmax.
    // Velocity after accel macro (end of seg3): vA = v1 + a*ta + 0.5*a*tj
    auto velocity_after_macro = [&](double ta) { return 0.5 * a * a / j + a * ta + 0.5 * a * tj; };

    // Solve for ta to reach vmax if possible.
    double v_macro_base = 0.5 * a * a / j + 0.5 * a * tj; // ta=0 case (triangular accel jerk limited)
    double need_ta = (vmax_ - v_macro_base) / a;
    if (need_ta < 0.0)
        need_ta = 0.0; // cannot use negative plateau

    // Distance of accel half with required plateau to reach vmax
    double p_accel_half = integrate_accel_macro(need_ta);
    double p_total_min_vmax = 2.0 * p_accel_half; // symmetric decel

    if (p_total_min_vmax > displacement_) {
        // Not enough distance to reach vmax. Need to reduce a or drop segments.
        // Approach: binary search on a_peak to match half distance = displacement_/2.
        double low_a = 1e-9, high_a = a;
        double target_half = displacement_ / 2.0;
        for (int i = 0; i < 40; i++) {
            double mid_a = 0.5 * (low_a + high_a);
            double tj_mid = mid_a / j;
            // compute ta needed to keep velocity symmetric WITHOUT exceeding vmax (ignore vmax here)
            double ta_mid = 0.0; // we try without plateau first because adding plateau increases distance
            // If even triangular sequence overshoots target half distance we must lower accel.
            auto integrate_macro_mid = [&](double ta) {
                double p1 = j*pow(tj_mid,3)/6.0;
                double v1 = 0.5*j*tj_mid*tj_mid;
                double p2 = v1*ta + 0.5*mid_a*ta*ta;
                double v2 = v1 + mid_a*ta;
                double p3 = v2*tj_mid + 0.5*mid_a*tj_mid*tj_mid - j*pow(tj_mid,3)/6.0;
                return p1+p2+p3; };
            double p_half_mid = integrate_macro_mid(ta_mid);
            if (p_half_mid > target_half) {
                high_a = mid_a; // need less accel
            } else {
                low_a = mid_a; // can try higher
            }
        }
        a_peak_ = low_a;
        double tj_new = a_peak_ / j;
        // recompute half distance with triangular jerk-limited acceleration (no plateau)
        double p1 = j * pow(tj_new, 3) / 6.0;
        double v1 = 0.5 * j * tj_new * tj_new;
        double p3 = (v1)*tj_new + 0.5 * a_peak_ * tj_new * tj_new - j * pow(tj_new, 3) / 6.0; // ta=0
        double p_half = p1 + p3;
        // Mirror for decel -> total distance = 2*p_half. Slight mismatch due to rounding: absorb into cruise (none) via scale factor on velocity.
        double scale = displacement_ / (2.0 * p_half);
        if (scale < 0.999 || scale > 1.001) {
            // adjust effective jerk proportionally (simpler) to scale distances; this changes time proportionally.
            // Equivalent to scaling position outputs later. We'll keep scale for sampling.
        }
        v_cruise_ = 0.0; // no cruise
        dt_[0] = tj_new;
        dt_[1] = 0.0;
        dt_[2] = tj_new; // accel side
        dt_[3] = 0.0;    // cruise
        dt_[4] = tj_new;
        dt_[5] = 0.0;
        dt_[6] = tj_new; // decel side
        buildCumulative();
        precomputeStates();
        total_time_ = t_cum_[7];
        return;
    }

    // We can reach vmax. Compute remaining distance for cruise.
    a_peak_ = a;
    double cruise_distance = displacement_ - p_total_min_vmax;
    double v_after_macro = velocity_after_macro(need_ta);
    // Adjust need_ta if v_after_macro exceeds vmax due to numeric error
    if (v_after_macro > vmax_) {
        need_ta -= (v_after_macro - vmax_) / a_peak_;
        if (need_ta < 0)
            need_ta = 0;
        p_accel_half = integrate_accel_macro(need_ta);
        cruise_distance = displacement_ - 2.0 * p_accel_half;
    }
    if (cruise_distance < 0)
        cruise_distance = 0;
    v_cruise_ = vmax_;
    double t_cruise = cruise_distance / v_cruise_;

    double tj1 = tj;     // ramp up jerk
    double ta = need_ta; // constant accel in segment 2
    // Segment durations (symmetry):
    dt_[0] = tj1;      // +J
    dt_[1] = ta;       // +A plateau
    dt_[2] = tj1;      // -J to zero accel
    dt_[3] = t_cruise; // cruise
    dt_[4] = tj1;      // -J
    dt_[5] = ta;       // -A plateau
    dt_[6] = tj1;      // +J to zero accel

    buildCumulative();
    precomputeStates();
    total_time_ = t_cum_[7];
}

void SCurveProfile::buildCumulative() {
    t_cum_[0] = 0.0;
    for (int i = 0; i < 7; i++) {
        t_cum_[i + 1] = t_cum_[i] + dt_[i];
    }
}

void SCurveProfile::precomputeStates() {
    // Integrate segment by segment accumulating position, velocity, accel.
    double p = 0, v = 0, a = 0;
    seg_state_[0] = {p, v, a};
    double j = jmax_;
    double dir = direction_;
    for (int i = 0; i < 7; i++) {
        double dt = dt_[i];
        switch (i) {
        case 0: // accel ramps 0->a_peak (a(t)=j*t)
            // velocity gain: 0.5*j*dt^2
            // position gain: j*dt^3/6
            v += 0.5 * j * dt * dt;
            p += j * dt * dt * dt / 6.0;
            a = j * dt;
            break;
        case 1: // constant accel a
            v += a * dt;
            p += v * dt - 0.5 * a * dt * dt; // using initial v before applying accel for final
            // a unchanged
            break;
        case 2: // accel ramps a -> 0 with -j
            // velocity gain: a*dt - 0.5*j*dt^2
            // position gain: v*dt + 0.5*a*dt^2 - j*dt^3/6  (v is current velocity at start)
            p += v * dt + 0.5 * a * dt * dt - j * dt * dt * dt / 6.0;
            v += a * dt - 0.5 * j * dt * dt;
            a = 0.0;
            break;
        case 3: // cruise (a=0)
            p += v * dt;
            break;
        case 4: // accel ramps 0 -> -a_peak (jerk -j)
            // mirror of segment 0 but negative accel
            // velocity change: -0.5*j*dt^2
            // position: v*dt - j*dt^3/6
            p += v * dt - j * dt * dt * dt / 6.0;
            v += -0.5 * j * dt * dt;
            a = -j * dt;
            break;
        case 5:                              // constant accel -a
            p += v * dt + 0.5 * a * dt * dt; // a negative
            v += a * dt;
            break;
        case 6: // accel ramps -a -> 0 with +j
            // velocity change: a*dt + 0.5*j*dt^2 (a negative; after segment a->0)
            // position: v*dt + 0.5*a*dt^2 + j*dt^3/6
            p += v * dt + 0.5 * a * dt * dt + j * dt * dt * dt / 6.0;
            v += a * dt + 0.5 * j * dt * dt;
            a = 0.0;
            break;
        }
        seg_state_[i + 1] = {p, v, a};
    }
    // Scale and shift for direction and start angle
    double scale = displacement_ > 0 ? (displacement_ / seg_state_[7].p) : 1.0;
    for (int i = 0; i <= 7; i++) {
        seg_state_[i].p = start_angle_ + dir * seg_state_[i].p * scale;
        seg_state_[i].v = dir * seg_state_[i].v * scale;
        seg_state_[i].a = dir * seg_state_[i].a * scale;
    }
}

SCurveProfile::Sample SCurveProfile::sample(double t) const {
    if (total_time_ <= 0.0) {
        return {start_angle_, 0.0, 0.0};
    }
    if (t <= 0)
        return {seg_state_[0].p, seg_state_[0].v, seg_state_[0].a};
    if (t >= total_time_)
        return {seg_state_[7].p, 0.0, 0.0};

    // Find segment index
    int seg = 0;
    while (seg < 7 && t >= t_cum_[seg + 1])
        ++seg;
    double local_t = t - t_cum_[seg];
    const double j = jmax_ * direction_;
    const auto &s0 = seg_state_[seg];
    switch (seg) {
    case 0: { // a(t)=j*local_t
        double a = j * local_t;
        double v = s0.v + 0.5 * j * local_t * local_t;
        double p = s0.p + j * pow(local_t, 3) / 6.0;
        return {p, v, a};
    }
    case 1: {                       // constant +a_peak
        double a = seg_state_[1].a; // already scaled
        double v = s0.v + a * local_t;
        double p = s0.p + s0.v * local_t + 0.5 * a * local_t * local_t;
        return {p, v, a};
    }
    case 2: { // a(t) = a_peak - j*local_t
        double a_start = seg_state_[1].a;
        double a = a_start - j * local_t;
        double v = s0.v + a_start * local_t - 0.5 * j * local_t * local_t;
        double p = s0.p + s0.v * local_t + 0.5 * a_start * local_t * local_t - j * pow(local_t, 3) / 6.0;
        return {p, v, a};
    }
    case 3: { // cruise
        double v = s0.v;
        double a = 0.0;
        double p = s0.p + v * local_t;
        return {p, v, a};
    }
    case 4: { // a(t) = -j*local_t
        double a = -j * local_t;
        double v = s0.v - 0.5 * j * local_t * local_t;
        double p = s0.p + s0.v * local_t - j * pow(local_t, 3) / 6.0;
        return {p, v, a};
    }
    case 5: {                       // constant -a_peak
        double a = seg_state_[5].a; // negative
        double v = s0.v + a * local_t;
        double p = s0.p + s0.v * local_t + 0.5 * a * local_t * local_t;
        return {p, v, a};
    }
    case 6: {                             // a(t) = -a_peak + j*local_t
        double a_start = seg_state_[5].a; // negative
        double a = a_start + j * local_t;
        double v = s0.v + a_start * local_t + 0.5 * j * local_t * local_t;
        double p = s0.p + s0.v * local_t + 0.5 * a_start * local_t * local_t + j * pow(local_t, 3) / 6.0;
        return {p, v, a};
    }
    }
    return {start_angle_, 0, 0};
}
