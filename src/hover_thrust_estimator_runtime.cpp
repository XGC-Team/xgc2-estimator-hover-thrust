#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "hover_thrust_estimator/state_machine/airborne_state.h"
#include "hover_thrust_estimator/state_machine/fault_state.h"
#include "hover_thrust_estimator/state_machine/ground_state.h"
#include "hover_thrust_estimator/state_machine/health_monitor_state.h"
#include "hover_thrust_estimator/state_machine/self_check_state.h"

namespace hover_thrust_estimator {
namespace {

namespace sm = state_machine;

constexpr double kDefaultSampleTimeout = 0.2;
constexpr double kDefaultOutputSlewRate = 0.1;
constexpr double kMinimumInputRateHz = 1.0e-3;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

bool targets(const HoverThrustEstimatorRuntime& runtime, HoverThrustRuntimeState state) {
    return runtime.targetState() == state;
}

}  // namespace

HoverThrustEstimatorRuntime::HoverThrustEstimatorRuntime() {
    normalizeConfig();
    reset();
}

void HoverThrustEstimatorRuntime::setConfig(const Config& config) {
    config_ = config;
    normalizeConfig();
    reset();
}

void HoverThrustEstimatorRuntime::reset() {
    estimator_.setConfig(HoverThrustEstimator::Config{
        config_.rho2, config_.min_hover_thrust, config_.max_hover_thrust, config_.filter_enabled,
        config_.filter_cutoff_hz});
    estimator_.reset(config_.gravity, config_.initial_hover_thrust);
    input_ = Input{};
    health_ = Classification{};
    hover_thrust_output_ = config_.initial_hover_thrust;
    raw_hover_thrust_output_ = config_.initial_hover_thrust;
    current_time_sec_ = 0.0;
    last_estimate_stamp_sec_ = 0.0;
    raw_update_requested_ = false;
    fault_requested_ = false;
    output_slew_limiter_.reset(config_.output_slew_rate, hover_thrust_output_);
    state_ = HoverThrustRuntimeState::kSelfCheck;
    flags_ = HoverThrustRuntimeFlag::kDegraded;
    last_output_ = makeOutput(state_, flags_, false, health_, current_time_sec_);
    setupMachine();
}

sm::Status HoverThrustEstimatorRuntime::postInputEvent(sm::Event event, const Input& input) {
    applyInputEvent(event.id, input);
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void HoverThrustEstimatorRuntime::postInputEvent(HoverThrustInputEvent event, const Input& input) {
    requireOk(postInputEvent(inputEvent(event, input.now_sec), input), "post input event");
}

void HoverThrustEstimatorRuntime::requestRawUpdate(double now_sec) {
    current_time_sec_ = now_sec;
    raw_update_requested_ = true;
    sm::Event event(event_type::INPUT_RAW_UPDATE_DUE, sm::EventTimestamp{now_sec});
    event.source = "raw_update_timer";
    event.category = sm::EventCategory::kInput;
    requireOk(machine_->postEvent(std::move(event)), "post raw update event");
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    refreshHealth();
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        fault_requested_ = true;
        state_ = HoverThrustRuntimeState::kFault;
        flags_ |= HoverThrustRuntimeFlag::kStateMachineFault;
        last_output_ = makeOutput(state_, flags_, false, health_, current_time_sec_);
    }
    return last_output_;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output(double now_sec) const {
    const Classification classification = classify(now_sec);
    return makeOutput(state_, flags_, false, classification, now_sec);
}

void HoverThrustEstimatorRuntime::refreshHealth() {
    health_ = classify(current_time_sec_);
}

void HoverThrustEstimatorRuntime::enterState(HoverThrustRuntimeState state) {
    state_ = state;
}

void HoverThrustEstimatorRuntime::performSelfCheck() {
    last_output_ = publishForState(HoverThrustRuntimeState::kSelfCheck, health_.flags, false);
}

void HoverThrustEstimatorRuntime::performGround() {
    last_output_ = publishForState(HoverThrustRuntimeState::kGround, health_.flags, false);
}

void HoverThrustEstimatorRuntime::performAirborne() {
    uint32_t target_flags = health_.flags;
    bool sample_used = false;

    if (raw_update_requested_) {
        if (health_.ready) {
            sample_used = estimator_.update(input_.imu_acc_z.value, input_.normalized_thrust.value,
                                            input_.normalized_thrust.stamp_sec);
            if (sample_used) {
                raw_hover_thrust_output_ = std::clamp(
                    estimator_.rawEstimate(), config_.min_hover_thrust, config_.max_hover_thrust);
                const double dt_s = last_estimate_stamp_sec_ > 0.0
                                        ? std::max(0.0, input_.normalized_thrust.stamp_sec -
                                                            last_estimate_stamp_sec_)
                                        : 0.0;
                const double filtered = std::clamp(estimator_.estimate(), config_.min_hover_thrust,
                                                   config_.max_hover_thrust);
                hover_thrust_output_ =
                    std::clamp(output_slew_limiter_.filter(filtered, dt_s),
                               config_.min_hover_thrust, config_.max_hover_thrust);
                last_estimate_stamp_sec_ = input_.normalized_thrust.stamp_sec;
                health_.estimate_valid = true;
            } else {
                target_flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
            }
        } else {
            target_flags |= HoverThrustRuntimeFlag::kRawEstimateStale;
        }
        raw_update_requested_ = false;
    }

    last_output_ = publishForState(HoverThrustRuntimeState::kAirborne, target_flags, sample_used);
}

void HoverThrustEstimatorRuntime::performFault() {
    raw_update_requested_ = false;
    last_output_ =
        publishForState(HoverThrustRuntimeState::kFault,
                        health_.flags | HoverThrustRuntimeFlag::kStateMachineFault, false);
}

void HoverThrustEstimatorRuntime::normalizeConfig() {
    config_.gravity =
        finitePositive(config_.gravity) ? config_.gravity : estimator_limits::kDefaultGravity;
    config_.min_hover_thrust = std::clamp(finiteOrDefault(config_.min_hover_thrust, 0.05), 0.0,
                                          estimator_limits::kMaximumNormalizedThrust);
    config_.max_hover_thrust =
        std::clamp(finiteOrDefault(config_.max_hover_thrust, 0.95), config_.min_hover_thrust,
                   estimator_limits::kMaximumNormalizedThrust);
    config_.initial_hover_thrust = std::clamp(finiteOrDefault(config_.initial_hover_thrust, 0.5),
                                              config_.min_hover_thrust, config_.max_hover_thrust);
    if (!std::isfinite(config_.rho2) || config_.rho2 <= 0.0 || config_.rho2 > 1.0) {
        config_.rho2 = 0.998;
    }
    if (!std::isfinite(config_.min_altitude)) {
        config_.min_altitude = 0.5;
    }
    if (!std::isfinite(config_.sample_timeout) || config_.sample_timeout <= 0.0) {
        config_.sample_timeout = kDefaultSampleTimeout;
    }
    if (!std::isfinite(config_.filter_cutoff_hz) || config_.filter_cutoff_hz <= 0.0) {
        config_.filter_cutoff_hz = 0.0;
        config_.filter_enabled = false;
    }
    if (!std::isfinite(config_.output_slew_rate) || config_.output_slew_rate < 0.0) {
        config_.output_slew_rate = kDefaultOutputSlewRate;
    }
    if (!std::isfinite(config_.input_rate_low_hz) || config_.input_rate_low_hz <= 0.0) {
        config_.input_rate_low_hz = 0.0;
    }
}

void HoverThrustEstimatorRuntime::setupMachine() {
    auto builder = sm::StateMachine::builder("HoverThrustEstimatorStateMachine");
    builder.region(region_type::HEALTH)
        .name("health")
        .order(0)
        .initial(state_type::HealthMonitor)
        .state(state_type::HealthMonitor)
        .name("HealthMonitor")
        .impl(std::make_unique<HealthMonitorState>(*this))
        .endRegion()
        .region(region_type::ESTIMATION)
        .name("estimation")
        .order(10)
        .initial(state_type::SelfCheck)
        .state(state_type::SelfCheck)
        .name("SelfCheck")
        .impl(std::make_unique<SelfCheckState>(*this))
        .state(state_type::Ground)
        .name("Ground")
        .impl(std::make_unique<GroundState>(*this))
        .state(state_type::Airborne)
        .name("Airborne")
        .impl(std::make_unique<AirborneState>(*this))
        .state(state_type::Fault)
        .name("Fault")
        .impl(std::make_unique<FaultState>(*this))
        .endRegion();

    const std::array<sm::StateId, 4> states{state_type::SelfCheck, state_type::Ground,
                                            state_type::Airborne, state_type::Fault};
    for (const sm::StateId from : states) {
        if (from != state_type::SelfCheck) {
            builder.transition()
                .from(from)
                .to(state_type::SelfCheck)
                .priority(transition_priority::AUTOMATIC)
                .when([this](const sm::GuardContext&) {
                    return targets(*this, HoverThrustRuntimeState::kSelfCheck);
                });
        }
        if (from != state_type::Ground) {
            builder.transition()
                .from(from)
                .to(state_type::Ground)
                .priority(transition_priority::AUTOMATIC)
                .when([this](const sm::GuardContext&) {
                    return targets(*this, HoverThrustRuntimeState::kGround);
                });
        }
        if (from != state_type::Airborne) {
            builder.transition()
                .from(from)
                .to(state_type::Airborne)
                .priority(transition_priority::AUTOMATIC)
                .when([this](const sm::GuardContext&) {
                    return targets(*this, HoverThrustRuntimeState::kAirborne);
                });
        }
        if (from != state_type::Fault) {
            builder.transition()
                .from(from)
                .to(state_type::Fault)
                .priority(transition_priority::FAULT)
                .when([this](const sm::GuardContext&) {
                    return targets(*this, HoverThrustRuntimeState::kFault);
                });
        }
    }

    auto machine_result = builder.build();
    requireOk(machine_result.status, "build hover thrust estimator state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start hover thrust estimator state machine");
}

sm::Event HoverThrustEstimatorRuntime::inputEvent(HoverThrustInputEvent event, double timestamp) {
    sm::Event result;
    switch (event) {
        case HoverThrustInputEvent::kImuUpdated:
            result.id = event_type::INPUT_IMU_UPDATED;
            result.source = "imu";
            break;
        case HoverThrustInputEvent::kThrustUpdated:
            result.id = event_type::INPUT_THRUST_UPDATED;
            result.source = "target_attitude";
            break;
        case HoverThrustInputEvent::kAltitudeUpdated:
            result.id = event_type::INPUT_ALTITUDE_UPDATED;
            result.source = "altitude";
            break;
    }
    result.timestamp = timestamp;
    result.category = sm::EventCategory::kInput;
    return result;
}

void HoverThrustEstimatorRuntime::applyInputEvent(sm::EventId event_id, const Input& input) {
    switch (event_id) {
        case event_type::INPUT_IMU_UPDATED:
            input_.imu_acc_z = input.imu_acc_z;
            break;
        case event_type::INPUT_THRUST_UPDATED:
            input_.normalized_thrust = input.normalized_thrust;
            input_.thrust_ignored = input.thrust_ignored;
            break;
        case event_type::INPUT_ALTITUDE_UPDATED:
            input_.altitude = input.altitude;
            break;
        default:
            break;
    }
}

HoverThrustEstimatorRuntime::Classification HoverThrustEstimatorRuntime::classify(
    double now_sec) const {
    Classification result;
    result.state = HoverThrustRuntimeState::kSelfCheck;
    result.flags = HoverThrustRuntimeFlag::kDegraded;
    result.degraded = true;

    if (!input_.imu_acc_z.received) {
        result.flags |= HoverThrustRuntimeFlag::kImuMissing;
    }
    if (!input_.normalized_thrust.received) {
        result.flags |= HoverThrustRuntimeFlag::kThrustMissing;
    }
    if (!input_.altitude.received) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeMissing;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuMissing | HoverThrustRuntimeFlag::kThrustMissing |
          HoverThrustRuntimeFlag::kAltitudeMissing)) != 0) {
        return result;
    }

    if (sampleTimeJumped(input_.imu_acc_z, now_sec) ||
        sampleTimeJumped(input_.normalized_thrust, now_sec) ||
        sampleTimeJumped(input_.altitude, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kTimeJump;
        return result;
    }
    if (sampleStale(input_.imu_acc_z, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kImuStale;
    }
    if (sampleStale(input_.normalized_thrust, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kThrustStale;
    }
    if (sampleStale(input_.altitude, now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeStale;
    }
    if (sampleRateLow(input_.imu_acc_z) || sampleRateLow(input_.normalized_thrust) ||
        sampleRateLow(input_.altitude)) {
        result.flags |= HoverThrustRuntimeFlag::kInputRateLow;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuStale | HoverThrustRuntimeFlag::kThrustStale |
          HoverThrustRuntimeFlag::kAltitudeStale | HoverThrustRuntimeFlag::kInputRateLow)) != 0) {
        return result;
    }

    if (!input_.imu_acc_z.finite) {
        result.flags |= HoverThrustRuntimeFlag::kImuInvalid;
    }
    if (!input_.altitude.finite) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeInvalid;
    }
    if (input_.thrust_ignored || !input_.normalized_thrust.finite ||
        input_.normalized_thrust.value <= estimator_limits::kMinimumNormalizedThrust ||
        input_.normalized_thrust.value > estimator_limits::kMaximumNormalizedThrust) {
        result.flags |= HoverThrustRuntimeFlag::kThrustInvalid;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuInvalid | HoverThrustRuntimeFlag::kThrustInvalid |
          HoverThrustRuntimeFlag::kAltitudeInvalid)) != 0) {
        return result;
    }

    const double altitude = input_.altitude.value;
    if (altitude < config_.min_altitude) {
        result.state = HoverThrustRuntimeState::kGround;
        result.flags |= HoverThrustRuntimeFlag::kBelowMinAltitude |
                        HoverThrustRuntimeFlag::kGroundHold | HoverThrustRuntimeFlag::kDegraded;
        result.degraded = true;
        result.estimate_valid = estimator_.valid();
        return result;
    }

    result.state = HoverThrustRuntimeState::kAirborne;
    result.flags = 0;
    result.ready = true;
    result.degraded = false;
    result.estimate_valid = estimator_.valid();
    result.source_stamp_sec = input_.normalized_thrust.stamp_sec;
    return result;
}

bool HoverThrustEstimatorRuntime::sampleStale(const Sample& sample, double now_sec) const {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    return now_sec - sample.stamp_sec > config_.sample_timeout;
}

bool HoverThrustEstimatorRuntime::sampleRateLow(const Sample& sample) const {
    if (config_.input_rate_low_hz <= 0.0 || !sample.received || !std::isfinite(sample.period_sec) ||
        sample.period_sec <= 0.0) {
        return false;
    }
    return 1.0 / sample.period_sec < std::max(config_.input_rate_low_hz, kMinimumInputRateHz);
}

bool HoverThrustEstimatorRuntime::sampleTimeJumped(const Sample& sample, double now_sec) const {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    if (sample.stamp_sec > now_sec + 0.05) {
        return true;
    }
    return std::isfinite(sample.period_sec) && sample.period_sec < -0.05;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::publishForState(
    HoverThrustRuntimeState state, uint32_t flags, bool sample_used) {
    if (state == HoverThrustRuntimeState::kFault) {
        flags |= HoverThrustRuntimeFlag::kStateMachineFault;
    }
    flags_ = flags;
    state_ = state;
    return makeOutput(state_, flags_, sample_used, health_, current_time_sec_);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::makeOutput(
    HoverThrustRuntimeState state, uint32_t flags, bool sample_used,
    const Classification& classification, double output_stamp_sec) const {
    Output output;
    output.state = state;
    output.flags = flags;
    output.hover_thrust = state == HoverThrustRuntimeState::kSelfCheck
                              ? config_.initial_hover_thrust
                              : hover_thrust_output_;
    output.raw_hover_thrust = raw_hover_thrust_output_;
    output.initial_hover_thrust = config_.initial_hover_thrust;
    output.thrust_to_acceleration = output.hover_thrust > estimator_limits::kMinimumNormalizedThrust
                                        ? config_.gravity / output.hover_thrust
                                        : config_.gravity / config_.initial_hover_thrust;
    output.degraded = classification.degraded || state != HoverThrustRuntimeState::kAirborne;
    output.estimate_valid = classification.estimate_valid && !output.degraded &&
                            state == HoverThrustRuntimeState::kAirborne && estimator_.valid();
    output.estimator_valid = output.estimate_valid;
    output.sample_used = sample_used;
    output.source_stamp_sec = classification.source_stamp_sec;
    output.last_estimate_stamp_sec = last_estimate_stamp_sec_;
    return output;
}

}  // namespace hover_thrust_estimator
