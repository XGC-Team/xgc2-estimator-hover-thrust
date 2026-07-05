#pragma once

#include <cmath>

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

   private:
    bool initialized_{false};
    double last_fire_sec_{0.0};
};

}  // namespace hover_thrust_estimator
