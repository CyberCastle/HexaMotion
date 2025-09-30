#include "../src/s_curve_profile.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

// Basic validation helper
static void expectNear(double a, double b, double eps, const char *msg) {
    if (std::fabs(a - b) > eps) {
        std::cerr << "FAIL: " << msg << " expected " << b << " got " << a << "\n";
        std::exit(1);
    }
}

int main() {
    {
        std::cout << "Test 1: Long move with cruise" << std::endl;
        SCurveProfile prof(0.0, 10.0, 3.0, 4.0, 20.0);
        assert(prof.valid());
        double T = prof.totalTime();
        assert(T > 0.0);
        // Sample at fine resolution
        double last_pos = -1e9;
        double last_vel = -1e9;
        double last_acc = 0.0;
        int steps = 2000;
        double max_acc_jump = 0.0;
        for (int i = 0; i <= steps; i++) {
            double t = (T * i) / steps;
            auto s = prof.sample(t);
            // Monotonic increasing position
            if (i > 0 && s.position < last_pos - 1e-6) {
                std::cerr << "Position decreased at i=" << i << "\n";
                return 1;
            }
            // Track acceleration discontinuities
            if (i > 0) {
                max_acc_jump = std::max(max_acc_jump, std::fabs(s.acceleration - last_acc));
            }
            last_pos = s.position;
            last_vel = s.velocity;
            last_acc = s.acceleration;
        }
        expectNear(last_pos, 10.0, 1e-5, "Final position mismatch");
        std::cout << "  Total time: " << T << " max acc jump: " << max_acc_jump << std::endl;
    }

    {
        std::cout << "Test 2: Short move (should collapse plateaus)" << std::endl;
        SCurveProfile prof(0.0, 0.05, 3.0, 4.0, 20.0);
        assert(prof.valid());
        double T = prof.totalTime();
        assert(T > 0.0);
        auto s_end = prof.sample(T);
        expectNear(s_end.position, 0.05, 1e-5, "Short move final position");
    }

    {
        std::cout << "Test 3: Reverse direction" << std::endl;
        SCurveProfile prof(5.0, -2.0, 2.0, 3.0, 15.0);
        assert(prof.valid());
        double T = prof.totalTime();
        auto s0 = prof.sample(0.0);
        auto sT = prof.sample(T);
        expectNear(s0.position, 5.0, 1e-9, "Reverse start pos");
        expectNear(sT.position, -2.0, 1e-6, "Reverse end pos");
        // Ensure position decreases
        double last_pos = 1e9;
        for (int i = 0; i <= 500; i++) {
            double t = (T * i) / 500.0;
            auto s = prof.sample(t);
            if (i > 0 && s.position > last_pos + 1e-6) {
                std::cerr << "Position increased in reverse move\n";
                return 1;
            }
            last_pos = s.position;
        }
    }

    {
        std::cout << "Test 4: Zero displacement" << std::endl;
        SCurveProfile prof(1.23, 1.23, 1.0, 1.0, 1.0);
        assert(prof.totalTime() == 0.0);
        auto s = prof.sample(0.5);
        expectNear(s.position, 1.23, 1e-12, "Zero displacement pos");
        expectNear(s.velocity, 0.0, 1e-12, "Zero displacement vel");
    }

    {
        std::cout << "Test 5: Parabolic fallback honors velocity limit" << std::endl;
        // Tiny move where the jerk-limited solution would overrun vmax.
        SCurveProfile prof(0.0, 1e-3, 0.05, 0.2, 5.0);
        assert(prof.valid());
        assert(prof.isParabolicFallback());
        expectNear(prof.cruiseVelocity(), 0.0, 1e-9, "Parabolic fallback has no cruise");

        double T = prof.totalTime();
        double max_vel = 0.0;
        for (int i = 0; i <= 200; ++i) {
            double t = (T * i) / 200.0;
            auto s = prof.sample(t);
            max_vel = std::max(max_vel, std::fabs(s.velocity));
        }
        if (max_vel > 0.05 + 1e-6) {
            std::cerr << "Parabolic fallback exceeded vmax: " << max_vel << std::endl;
            return 1;
        }
    }

    {
        std::cout << "Test 6: Accessors expose peak kinematics" << std::endl;
        SCurveProfile prof(0.0, 2.0, 1.0, 0.8, 10.0);
        assert(prof.valid());
        double peak_v = prof.peakVelocity();
        // For this configuration the profile should reach cruise velocity.
        expectNear(peak_v, prof.cruiseVelocity(), 1e-9, "Peak velocity should equal cruise velocity when cruise present");
        auto durations = prof.segmentDurations();
        double accumulated = 0.0;
        for (double d : durations) {
            accumulated += d;
        }
        expectNear(accumulated, prof.totalTime(), 1e-6, "Segment durations must sum to total time");
    }

    std::cout << "All S-curve profile tests passed." << std::endl;
    return 0;
}
