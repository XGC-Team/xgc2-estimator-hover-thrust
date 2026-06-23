#pragma once

#include <algorithm>
#include <cmath>

#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator::config_utils {

constexpr double kDefaultInitialHoverThrust = 0.3;
constexpr double kDefaultRho2 = 0.998;
constexpr double kDefaultMinHoverThrust = 0.15;
constexpr double kDefaultMaxHoverThrust = 0.85;
constexpr double kDefaultMinAltitude = 0.5;
constexpr double kDefaultSampleTimeout = 0.2;
constexpr double kDefaultFilterCutoffHz = 2.0;
constexpr double kDefaultInputRateLowHz = 5.0;
constexpr double kDefaultPublishRateHz = 100.0;
constexpr double kDefaultRawUpdateRateHz = 10.0;

inline bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

inline double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

inline HoverThrustEstimatorConfig normalizeConfig(HoverThrustEstimatorConfig config) {
    config.gravity =
        finitePositive(config.gravity) ? config.gravity : estimator_limits::kDefaultGravity;
    config.min_hover_thrust =
        std::clamp(finiteOrDefault(config.min_hover_thrust, kDefaultMinHoverThrust), 0.0,
                   estimator_limits::kMaximumNormalizedThrust);
    config.max_hover_thrust =
        std::clamp(finiteOrDefault(config.max_hover_thrust, kDefaultMaxHoverThrust),
                   config.min_hover_thrust, estimator_limits::kMaximumNormalizedThrust);
    config.initial_hover_thrust =
        std::clamp(finiteOrDefault(config.initial_hover_thrust, kDefaultInitialHoverThrust),
                   config.min_hover_thrust, config.max_hover_thrust);
    if (!std::isfinite(config.rho2) || config.rho2 <= 0.0 || config.rho2 > 1.0) {
        config.rho2 = kDefaultRho2;
    }
    if (!std::isfinite(config.min_altitude)) {
        config.min_altitude = kDefaultMinAltitude;
    }
    if (!std::isfinite(config.sample_timeout) || config.sample_timeout <= 0.0) {
        config.sample_timeout = kDefaultSampleTimeout;
    }
    if (!std::isfinite(config.filter_cutoff_hz) || config.filter_cutoff_hz <= 0.0) {
        config.filter_cutoff_hz = 0.0;
        config.filter_enabled = false;
    }
    if (!std::isfinite(config.input_rate_low_hz) || config.input_rate_low_hz < 0.0) {
        config.input_rate_low_hz = kDefaultInputRateLowHz;
    }
    if (!std::isfinite(config.publish_rate_hz) || config.publish_rate_hz <= 0.0) {
        config.publish_rate_hz = kDefaultPublishRateHz;
    }
    if (!std::isfinite(config.raw_update_rate_hz) || config.raw_update_rate_hz <= 0.0) {
        config.raw_update_rate_hz = kDefaultRawUpdateRateHz;
    }
    return config;
}

}  // namespace hover_thrust_estimator::config_utils
