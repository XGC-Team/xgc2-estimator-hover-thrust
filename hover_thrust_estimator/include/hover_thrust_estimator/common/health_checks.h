#pragma once

#include <cmath>
#include <xgc2_math/utils/sample_timing.hpp>

#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator::health_checks {

inline HoverThrustHealthStatus classify(const HoverThrustInput& input,
                                        const HoverThrustEstimatorConfig& config, double now_sec) {
    HoverThrustHealthStatus result;
    result.condition = HoverThrustHealthCondition::kInputUnhealthy;
    result.flags = 0;

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

    if (xgc2_math::sampleTimeJumped(now_sec, input.imu_acc_z.stamp_sec,
                                    input.imu_acc_z.period_sec) ||
        xgc2_math::sampleTimeJumped(now_sec, input.normalized_thrust.stamp_sec,
                                    input.normalized_thrust.period_sec) ||
        xgc2_math::sampleTimeJumped(now_sec, input.altitude.stamp_sec, input.altitude.period_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kTimeJump;
        return result;
    }
    if (xgc2_math::sampleStale(now_sec, input.imu_acc_z.stamp_sec, config.sample_timeout)) {
        result.flags |= HoverThrustRuntimeFlag::kImuStale;
    }
    if (xgc2_math::sampleStale(now_sec, input.normalized_thrust.stamp_sec, config.sample_timeout)) {
        result.flags |= HoverThrustRuntimeFlag::kThrustStale;
    }
    if (xgc2_math::sampleStale(now_sec, input.altitude.stamp_sec, config.sample_timeout)) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeStale;
    }
    if (xgc2_math::sampleRateLow(input.imu_acc_z.received, input.imu_acc_z.period_sec,
                                 config.input_rate_low_hz) ||
        xgc2_math::sampleRateLow(input.normalized_thrust.received,
                                 input.normalized_thrust.period_sec, config.input_rate_low_hz) ||
        xgc2_math::sampleRateLow(input.altitude.received, input.altitude.period_sec,
                                 config.input_rate_low_hz)) {
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
        result.condition = HoverThrustHealthCondition::kBelowMinAltitude;
        result.flags |= HoverThrustRuntimeFlag::kBelowMinAltitude;
        return result;
    }

    result.condition = HoverThrustHealthCondition::kEstimationReady;
    result.ready = true;
    result.source_stamp_sec = input.normalized_thrust.stamp_sec;
    return result;
}

}  // namespace hover_thrust_estimator::health_checks
