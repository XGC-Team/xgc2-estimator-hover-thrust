#pragma once

#include <algorithm>
#include <cmath>
#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator::health_checks {

constexpr double kMinimumInputRateHz = 1.0e-3;

inline bool sampleStale(const HoverThrustSample& sample, double now_sec,
                        const HoverThrustEstimatorConfig& config) {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    return now_sec - sample.stamp_sec > config.sample_timeout;
}

inline bool sampleRateLow(const HoverThrustSample& sample,
                          const HoverThrustEstimatorConfig& config) {
    const double minimum_rate = config.input_rate_low_hz;
    if (minimum_rate <= 0.0 || !sample.received || !std::isfinite(sample.period_sec) ||
        sample.period_sec <= 0.0) {
        return false;
    }
    return 1.0 / sample.period_sec < std::max(minimum_rate, kMinimumInputRateHz);
}

inline bool sampleTimeJumped(const HoverThrustSample& sample, double now_sec) {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    if (sample.stamp_sec > now_sec + 0.05) {
        return true;
    }
    return std::isfinite(sample.period_sec) && sample.period_sec < -0.05;
}

inline HoverThrustHealthStatus classify(const HoverThrustInput& input,
                                        const HoverThrustEstimatorConfig& config,
                                        bool fault_requested, double now_sec) {
    HoverThrustHealthStatus result;
    result.state = state_type::SelfCheck;
    result.transition_event = event_type::HEALTH_TO_SELF_CHECK;
    result.flags = 0;

    if (fault_requested) {
        result.state = state_type::Fault;
        result.transition_event = event_type::HEALTH_TO_FAULT;
        result.flags |= HoverThrustRuntimeFlag::kStateMachineFault;
        return result;
    }

    if (!input.imu_acc_z.received) {
        result.flags |= HoverThrustRuntimeFlag::kImuMissing;
    }
    if (!input.normalized_thrust.received) {
        result.flags |= HoverThrustRuntimeFlag::kThrustMissing;
    }
    if (!input.altitude.received) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeMissing;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuMissing | HoverThrustRuntimeFlag::kThrustMissing |
          HoverThrustRuntimeFlag::kAltitudeMissing)) != 0) {
        return result;
    }

    if (sampleTimeJumped(input.imu_acc_z, now_sec) ||
        sampleTimeJumped(input.normalized_thrust, now_sec) ||
        sampleTimeJumped(input.altitude, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kTimeJump;
        return result;
    }
    if (sampleStale(input.imu_acc_z, now_sec, config)) {
        result.flags |= HoverThrustRuntimeFlag::kImuStale;
    }
    if (sampleStale(input.normalized_thrust, now_sec, config)) {
        result.flags |= HoverThrustRuntimeFlag::kThrustStale;
    }
    if (sampleStale(input.altitude, now_sec, config)) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeStale;
    }
    if (sampleRateLow(input.imu_acc_z, config) || sampleRateLow(input.normalized_thrust, config) ||
        sampleRateLow(input.altitude, config)) {
        result.flags |= HoverThrustRuntimeFlag::kInputRateLow;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuStale | HoverThrustRuntimeFlag::kThrustStale |
          HoverThrustRuntimeFlag::kAltitudeStale | HoverThrustRuntimeFlag::kInputRateLow)) != 0) {
        return result;
    }

    if (!input.imu_acc_z.finite) {
        result.flags |= HoverThrustRuntimeFlag::kImuInvalid;
    }
    if (!input.altitude.finite) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeInvalid;
    }
    if (input.thrust_ignored || !input.normalized_thrust.finite ||
        input.normalized_thrust.value <= estimator_limits::kMinimumNormalizedThrust ||
        input.normalized_thrust.value > estimator_limits::kMaximumNormalizedThrust) {
        result.flags |= HoverThrustRuntimeFlag::kThrustInvalid;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuInvalid | HoverThrustRuntimeFlag::kThrustInvalid |
          HoverThrustRuntimeFlag::kAltitudeInvalid)) != 0) {
        return result;
    }

    if (input.altitude.value < config.min_altitude) {
        result.state = state_type::Ground;
        result.transition_event = event_type::HEALTH_TO_GROUND;
        result.flags |=
            HoverThrustRuntimeFlag::kBelowMinAltitude | HoverThrustRuntimeFlag::kGroundHold;
        return result;
    }

    result.state = state_type::Airborne;
    result.transition_event = event_type::HEALTH_TO_AIRBORNE;
    result.ready = true;
    result.source_stamp_sec = input.normalized_thrust.stamp_sec;
    return result;
}

}  // namespace hover_thrust_estimator::health_checks
