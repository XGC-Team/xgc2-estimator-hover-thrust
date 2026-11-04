#pragma once

#include <cmath>
#include <optional>

namespace hover_thrust_estimator {

class PeriodicGate {
   public:
    void reset() {
        initialized_ = false;
        last_fire_sec_ = 0.0;
    }

    bool due(double now_sec, double period_sec) {
        if (!std::isfinite(now_sec) || !std::isfinite(period_sec) || period_sec <= 0.0) {
            return false;
        }
        if (!initialized_ || now_sec + 0.05 < last_fire_sec_) {
            last_fire_sec_ = now_sec;
            initialized_ = true;
            return true;
        }
        if (now_sec - last_fire_sec_ >= period_sec) {
            last_fire_sec_ = now_sec;
            return true;
        }
        return false;
    }

    // Read-only deadline projection for the ROS owner loop. It mirrors due()
    // exactly: an uninitialized or regressed gate is due now, otherwise the
    // next fire is one period after the last fire. due() itself remains the
    // only mutating decision.
    std::optional<double> nextDue(double now_sec, double period_sec) const {
        if (!std::isfinite(now_sec) || !std::isfinite(period_sec) || period_sec <= 0.0) {
            return std::nullopt;
        }
        if (!initialized_ || now_sec + 0.05 < last_fire_sec_) {
            return now_sec;
        }
        const double deadline = last_fire_sec_ + period_sec;
        if (!std::isfinite(deadline)) {
            return std::nullopt;
        }
        return deadline;
    }

   private:
    bool initialized_{false};
    double last_fire_sec_{0.0};
};

}  // namespace hover_thrust_estimator
