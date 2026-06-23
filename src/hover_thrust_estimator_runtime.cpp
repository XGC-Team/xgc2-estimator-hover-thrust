#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace hover_thrust_estimator {
namespace {

namespace sm = state_machine;

constexpr sm::StateId kRootState = 100;
constexpr double kDefaultSampleTimeout = 0.2;
constexpr double kDefaultOutputSlewRate = 0.1;
constexpr double kMinimumInputRateHz = 1.0e-3;

constexpr std::array<HoverThrustRuntimeState, 4> kRuntimeStates{
    HoverThrustRuntimeState::kSelfCheck, HoverThrustRuntimeState::kGround,
    HoverThrustRuntimeState::kAirborne, HoverThrustRuntimeState::kFault};

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

class RuntimeState final : public sm::State {
   public:
    explicit RuntimeState(std::string name) : name_(std::move(name)) {}

    std::string name() const override {
        return name_;
    }

   private:
    std::string name_;
};

bool finitePositive(double value) {
    return std::isfinite(value) && value > 0.0;
}

double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
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
    hover_thrust_output_ = config_.initial_hover_thrust;
    raw_hover_thrust_output_ = config_.initial_hover_thrust;
    last_estimate_stamp_sec_ = 0.0;
    output_slew_limiter_.reset(config_.output_slew_rate, hover_thrust_output_);
    state_ = HoverThrustRuntimeState::kSelfCheck;
    flags_ = HoverThrustRuntimeFlag::kDegraded;
    setupMachine();
}

void HoverThrustEstimatorRuntime::postInputEvent(HoverThrustInputEvent event, const Input& input) {
    switch (event) {
        case HoverThrustInputEvent::kImuUpdated:
            input_.imu_acc_z = input.imu_acc_z;
            break;
        case HoverThrustInputEvent::kThrustUpdated:
            input_.normalized_thrust = input.normalized_thrust;
            input_.thrust_ignored = input.thrust_ignored;
            break;
        case HoverThrustInputEvent::kAltitudeUpdated:
            input_.altitude = input.altitude;
            break;
    }
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(
    HoverThrustOutputEvent event, double now_sec) {
    Classification classification = classify(now_sec);
    bool sample_used = false;
    uint32_t target_flags = classification.flags;

    if (event == HoverThrustOutputEvent::kUpdateRawEstimate && classification.ready) {
        sample_used = estimator_.update(input_.imu_acc_z.value, input_.normalized_thrust.value,
                                        input_.normalized_thrust.stamp_sec);
        if (sample_used) {
            raw_hover_thrust_output_ = std::clamp(
                estimator_.rawEstimate(), config_.min_hover_thrust, config_.max_hover_thrust);
            const double dt_s =
                last_estimate_stamp_sec_ > 0.0
                    ? std::max(0.0, input_.normalized_thrust.stamp_sec - last_estimate_stamp_sec_)
                    : 0.0;
            const double filtered = std::clamp(estimator_.estimate(), config_.min_hover_thrust,
                                               config_.max_hover_thrust);
            hover_thrust_output_ = std::clamp(output_slew_limiter_.filter(filtered, dt_s),
                                              config_.min_hover_thrust, config_.max_hover_thrust);
            last_estimate_stamp_sec_ = input_.normalized_thrust.stamp_sec;
            classification.estimate_valid = true;
        } else {
            target_flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
        }
    } else if (event == HoverThrustOutputEvent::kUpdateRawEstimate &&
               classification.state == HoverThrustRuntimeState::kAirborne &&
               !classification.ready) {
        target_flags |= HoverThrustRuntimeFlag::kRawEstimateStale;
    }

    transitionTo(classification.state);
    if (state_ == HoverThrustRuntimeState::kFault) {
        target_flags |= HoverThrustRuntimeFlag::kStateMachineFault;
    }
    flags_ = target_flags;
    return makeOutput(state_, flags_, sample_used, classification, now_sec);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output(double now_sec) const {
    const Classification classification = classify(now_sec);
    return makeOutput(state_, flags_, false, classification, now_sec);
}

std::string HoverThrustEstimatorRuntime::stateName(HoverThrustRuntimeState state) {
    switch (state) {
        case HoverThrustRuntimeState::kSelfCheck:
            return "self_check";
        case HoverThrustRuntimeState::kGround:
            return "ground";
        case HoverThrustRuntimeState::kAirborne:
            return "airborne";
        case HoverThrustRuntimeState::kFault:
            return "fault";
    }
    return "fault";
}

sm::StateId HoverThrustEstimatorRuntime::stateId(HoverThrustRuntimeState state) {
    return 101u + static_cast<sm::StateId>(state);
}

sm::EventId HoverThrustEstimatorRuntime::eventId(HoverThrustRuntimeState state) {
    return 201u + static_cast<sm::EventId>(state);
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
    auto builder = sm::StateMachine::builder("hover_thrust_estimator_runtime");
    builder.region(sm::kDefaultRegion)
        .initial(kRootState)
        .state(kRootState)
        .name("root")
        .impl(std::make_unique<RuntimeState>("root"))
        .initial(stateId(HoverThrustRuntimeState::kSelfCheck));
    for (const HoverThrustRuntimeState state : kRuntimeStates) {
        builder.state(stateId(state))
            .name(stateName(state))
            .impl(std::make_unique<RuntimeState>(stateName(state)));
    }
    for (const HoverThrustRuntimeState state : kRuntimeStates) {
        builder.transition().from(kRootState).to(stateId(state)).on(eventId(state)).global();
    }
    auto machine_result = builder.build();
    requireOk(machine_result.status, "build runtime state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start runtime state machine");
}

void HoverThrustEstimatorRuntime::transitionTo(HoverThrustRuntimeState state) {
    if (state == state_) {
        return;
    }

    sm::Status status = machine_->postEvent(sm::Event(eventId(state)));
    if (status.ok()) {
        const auto update = machine_->update({4, 4, false});
        status = update.status;
    }

    if (!status.ok()) {
        state_ = HoverThrustRuntimeState::kFault;
        flags_ |= HoverThrustRuntimeFlag::kStateMachineFault;
        return;
    }
    state_ = state;
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
