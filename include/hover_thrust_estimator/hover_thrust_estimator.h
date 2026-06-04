#pragma once

#include <algorithm>
#include <cmath>
#include <xgc2_observer/recursive_least_squares.hpp>

namespace hover_thrust_estimator {
namespace estimator_limits {

constexpr double kDefaultGravity = 9.8066;
constexpr double kMinimumGravity = 1.0e-6;
constexpr double kMinimumNormalizedThrust = 1.0e-6;
constexpr double kMaximumNormalizedThrust = 1.0;

}  // namespace estimator_limits

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
        gravity_ = std::isfinite(gravity) && gravity > estimator_limits::kMinimumGravity
                       ? gravity
                       : estimator_limits::kDefaultGravity;
        hover_thrust_estimate_ =
            std::clamp(initial_hover_thrust, config_.min_hover_thrust, config_.max_hover_thrust);
        const double initial_thr2acc =
            hover_thrust_estimate_ > estimator_limits::kMinimumNormalizedThrust
                ? gravity_ / hover_thrust_estimate_
                : gravity_ / 0.5;
        xgc2_observer::ScalarRecursiveLeastSquaresOptions options;
        options.forgetting_factor = config_.rho2;
        options.initial_covariance = 100.0;
        options.min_abs_regressor = estimator_limits::kMinimumNormalizedThrust;
        rls_.setOptions(options);
        rls_.reset(initial_thr2acc);
        last_time_sec_ = 0.0;
        valid_ = false;
    }

    bool update(double acc_z, double normalized_thrust, double time_sec) {
        if (!std::isfinite(acc_z) || !std::isfinite(normalized_thrust) ||
            normalized_thrust <= estimator_limits::kMinimumNormalizedThrust ||
            normalized_thrust > estimator_limits::kMaximumNormalizedThrust ||
            !std::isfinite(time_sec)) {
            return false;
        }
        if (last_time_sec_ > 0.0 && time_sec <= last_time_sec_) {
            return false;
        }

        const auto sample = rls_.update(acc_z, normalized_thrust);
        if (!sample.measurement_accepted) {
            return false;
        }

        const double thr2acc = sample.parameter;
        if (std::isfinite(thr2acc) && thr2acc > estimator_limits::kMinimumGravity) {
            hover_thrust_estimate_ =
                std::clamp(gravity_ / thr2acc, config_.min_hover_thrust, config_.max_hover_thrust);
            valid_ = true;
            last_time_sec_ = time_sec;
            return true;
        }
        return false;
    }

    double estimate() const {
        return hover_thrust_estimate_;
    }
    bool valid() const {
        return valid_;
    }
    double lastTimeSec() const {
        return last_time_sec_;
    }

   private:
    Config config_{};
    double gravity_{9.8066};
    xgc2_observer::ScalarRecursiveLeastSquares rls_{};
    double hover_thrust_estimate_{0.5};
    double last_time_sec_{0.0};
    bool valid_{false};
};

}  // namespace hover_thrust_estimator
