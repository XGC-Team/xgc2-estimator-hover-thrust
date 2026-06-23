#include "hover_thrust_estimator/state_machine/health_monitor_state.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kMinimumInputRateHz = 1.0e-3;

double eventTimeOrCurrent(const ::state_machine::Event& event,
                          const HoverThrustEstimatorRuntime& runtime) {
    return event.timestamp > 0.0 ? event.timestamp : runtime.currentTime();
}

::state_machine::StateId stateIdFor(HoverThrustRuntimeState state) {
    switch (state) {
        case HoverThrustRuntimeState::kSelfCheck:
            return state_type::SelfCheck;
        case HoverThrustRuntimeState::kGround:
            return state_type::Ground;
        case HoverThrustRuntimeState::kAirborne:
            return state_type::Airborne;
        case HoverThrustRuntimeState::kFault:
            return state_type::Fault;
    }
    return state_type::Fault;
}

}  // namespace

HealthMonitorState::HealthMonitorState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onEvent(::state_machine::StateContext& ctx,
                                                          const ::state_machine::Event& event) {
    evaluateAndPostTransition(ctx, eventTimeOrCurrent(event, runtime_));
    return {};
}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext&) {
    runtime_.setHealth(classify(runtime_.currentTime()));
    return {};
}

void HealthMonitorState::evaluateAndPostTransition(::state_machine::StateContext& ctx,
                                                   double now_sec) const {
    const HealthStatus health = classify(now_sec);
    runtime_.setHealth(health);

    const ::state_machine::StateId active_state = ctx.currentState(region_type::ESTIMATION);
    const auto desired_event = transitionEventFor(health.state);
    if (desired_event == 0 || active_state == stateIdFor(health.state)) {
        return;
    }

    ::state_machine::Event event(desired_event, ::state_machine::EventTimestamp{now_sec});
    event.source = "health_monitor_state";
    event.category = ::state_machine::EventCategory::kInternal;
    const auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        runtime_.setHealth(HealthStatus{HoverThrustRuntimeState::kFault,
                                        health.flags | HoverThrustRuntimeFlag::kStateMachineFault,
                                        false, health.source_stamp_sec});
    }
}

HealthMonitorState::HealthStatus HealthMonitorState::classify(double now_sec) const {
    HealthStatus result;
    result.state = HoverThrustRuntimeState::kSelfCheck;
    result.flags = 0;

    if (runtime_.faultRequested()) {
        result.state = HoverThrustRuntimeState::kFault;
        result.flags |= HoverThrustRuntimeFlag::kStateMachineFault;
        return result;
    }

    const auto& input = runtime_.input();
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
    if (sampleStale(input.imu_acc_z, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kImuStale;
    }
    if (sampleStale(input.normalized_thrust, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kThrustStale;
    }
    if (sampleStale(input.altitude, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeStale;
    }
    if (sampleRateLow(input.imu_acc_z) || sampleRateLow(input.normalized_thrust) ||
        sampleRateLow(input.altitude)) {
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

    if (input.altitude.value < runtime_.config().min_altitude) {
        result.state = HoverThrustRuntimeState::kGround;
        result.flags |=
            HoverThrustRuntimeFlag::kBelowMinAltitude | HoverThrustRuntimeFlag::kGroundHold;
        return result;
    }

    result.state = HoverThrustRuntimeState::kAirborne;
    result.ready = true;
    result.source_stamp_sec = input.normalized_thrust.stamp_sec;
    return result;
}

bool HealthMonitorState::sampleStale(const HoverThrustEstimatorRuntime::Sample& sample,
                                     double now_sec) const {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    return now_sec - sample.stamp_sec > runtime_.config().sample_timeout;
}

bool HealthMonitorState::sampleRateLow(const HoverThrustEstimatorRuntime::Sample& sample) const {
    const double minimum_rate = runtime_.config().input_rate_low_hz;
    if (minimum_rate <= 0.0 || !sample.received || !std::isfinite(sample.period_sec) ||
        sample.period_sec <= 0.0) {
        return false;
    }
    return 1.0 / sample.period_sec < std::max(minimum_rate, kMinimumInputRateHz);
}

bool HealthMonitorState::sampleTimeJumped(const HoverThrustEstimatorRuntime::Sample& sample,
                                          double now_sec) const {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    if (sample.stamp_sec > now_sec + 0.05) {
        return true;
    }
    return std::isfinite(sample.period_sec) && sample.period_sec < -0.05;
}

::state_machine::EventId HealthMonitorState::transitionEventFor(HoverThrustRuntimeState state) {
    switch (state) {
        case HoverThrustRuntimeState::kSelfCheck:
            return event_type::HEALTH_TO_SELF_CHECK;
        case HoverThrustRuntimeState::kGround:
            return event_type::HEALTH_TO_GROUND;
        case HoverThrustRuntimeState::kAirborne:
            return event_type::HEALTH_TO_AIRBORNE;
        case HoverThrustRuntimeState::kFault:
            return event_type::HEALTH_TO_FAULT;
    }
    return 0;
}

}  // namespace hover_thrust_estimator
