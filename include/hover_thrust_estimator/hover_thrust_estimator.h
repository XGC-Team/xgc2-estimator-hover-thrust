#pragma once

#include <algorithm>
#include <cmath>

namespace hover_thrust_estimator {

class HoverThrustEstimator {
public:
    struct Config {
        double rho2{0.998};
        double min_hover_thrust{0.05};
        double max_hover_thrust{0.95};
    };

    void setConfig(const Config& config) {
        config_ = config;
        if (!std::isfinite(config_.rho2) || config_.rho2 <= 0.0 || config_.rho2 > 1.0) {
            config_.rho2 = 0.998;
        }
        config_.min_hover_thrust = std::clamp(config_.min_hover_thrust, 0.0, 1.0);
        config_.max_hover_thrust =
            std::clamp(config_.max_hover_thrust, config_.min_hover_thrust, 1.0);
    }

    void reset(double gravity, double initial_hover_thrust) {
        gravity_ = std::isfinite(gravity) && gravity > 1e-6 ? gravity : 9.8066;
        hover_thrust_estimate_ = std::clamp(initial_hover_thrust,
                                            config_.min_hover_thrust,
                                            config_.max_hover_thrust);
        thr2acc_ = hover_thrust_estimate_ > 1e-6
                       ? gravity_ / hover_thrust_estimate_
                       : gravity_ / 0.5;
        covariance_ = 100.0;
        last_time_sec_ = 0.0;
        valid_ = false;
    }

    bool update(double acc_z, double normalized_thrust, double time_sec) {
        if (!std::isfinite(acc_z) ||
            !std::isfinite(normalized_thrust) ||
            normalized_thrust <= 1e-6 ||
            !std::isfinite(time_sec)) {
            return false;
        }
        if (last_time_sec_ > 0.0 && time_sec <= last_time_sec_) {
            return false;
        }

        const double gamma = 1.0 / (config_.rho2 +
                                    normalized_thrust * covariance_ * normalized_thrust);
        const double gain = gamma * covariance_ * normalized_thrust;
        thr2acc_ = thr2acc_ + gain * (acc_z - normalized_thrust * thr2acc_);
        covariance_ = (1.0 - gain * normalized_thrust) * covariance_ / config_.rho2;

        if (std::isfinite(thr2acc_) && thr2acc_ > 1e-6) {
            hover_thrust_estimate_ = std::clamp(gravity_ / thr2acc_,
                                                config_.min_hover_thrust,
                                                config_.max_hover_thrust);
            valid_ = true;
            last_time_sec_ = time_sec;
            return true;
        }
        return false;
    }

    double estimate() const { return hover_thrust_estimate_; }
    bool valid() const { return valid_; }
    double lastTimeSec() const { return last_time_sec_; }

private:
    Config config_{};
    double gravity_{9.8066};
    double thr2acc_{9.8066 / 0.5};
    double covariance_{100.0};
    double hover_thrust_estimate_{0.5};
    double last_time_sec_{0.0};
    bool valid_{false};
};

}  // namespace hover_thrust_estimator
