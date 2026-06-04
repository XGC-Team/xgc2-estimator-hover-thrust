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

constexpr std::array<HoverThrustRuntimeState, 10> kRuntimeStates{
    HoverThrustRuntimeState::kInitializing,      HoverThrustRuntimeState::kEstimating,
    HoverThrustRuntimeState::kHoldingInputStale, HoverThrustRuntimeState::kWaitingForImu,
    HoverThrustRuntimeState::kWaitingForThrust,  HoverThrustRuntimeState::kWaitingForAltitude,
    HoverThrustRuntimeState::kBelowMinAltitude,  HoverThrustRuntimeState::kInvalidThrust,
    HoverThrustRuntimeState::kEstimatorRejected, HoverThrustRuntimeState::kFault};

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
    hover_thrust_output_ = config_.initial_hover_thrust;
    raw_hover_thrust_output_ = config_.initial_hover_thrust;
    last_estimate_stamp_sec_ = 0.0;
    state_ = HoverThrustRuntimeState::kInitializing;
    flags_ = 0;
    setupMachine();
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(const Input& input) {
    const Classification classification = classify(input);
    bool sample_used = false;
    HoverThrustRuntimeState target_state = classification.state;
    uint32_t target_flags = classification.flags;

    if (classification.ready) {
        sample_used = estimator_.update(input.imu_acc_z.value, input.normalized_thrust.value,
                                        input.normalized_thrust.stamp_sec);
        if (sample_used) {
            hover_thrust_output_ = estimator_.estimate();
            raw_hover_thrust_output_ = estimator_.rawEstimate();
            last_estimate_stamp_sec_ = input.normalized_thrust.stamp_sec;
            target_state = HoverThrustRuntimeState::kEstimating;
        } else {
            target_state = HoverThrustRuntimeState::kEstimatorRejected;
            target_flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
        }
    }

    transitionTo(target_state);
    if (state_ == HoverThrustRuntimeState::kFault) {
        target_flags |= HoverThrustRuntimeFlag::kStateMachineFault;
    }
    flags_ = target_flags;
    return makeOutput(state_, flags_, sample_used, classification.source_stamp_sec);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output() const {
    return makeOutput(state_, flags_, false, 0.0);
}

std::string HoverThrustEstimatorRuntime::stateName(HoverThrustRuntimeState state) {
    switch (state) {
        case HoverThrustRuntimeState::kInitializing:
            return "initializing";
        case HoverThrustRuntimeState::kEstimating:
            return "estimating";
        case HoverThrustRuntimeState::kHoldingInputStale:
            return "holding_input_stale";
        case HoverThrustRuntimeState::kWaitingForImu:
            return "waiting_for_imu";
        case HoverThrustRuntimeState::kWaitingForThrust:
            return "waiting_for_thrust";
        case HoverThrustRuntimeState::kWaitingForAltitude:
            return "waiting_for_altitude";
        case HoverThrustRuntimeState::kBelowMinAltitude:
            return "below_min_altitude";
        case HoverThrustRuntimeState::kInvalidThrust:
            return "invalid_thrust";
        case HoverThrustRuntimeState::kEstimatorRejected:
            return "estimator_rejected";
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
}

void HoverThrustEstimatorRuntime::setupMachine() {
    machine_ = std::make_unique<sm::StateMachine>("hover_thrust_estimator_runtime");
    requireOk(machine_->addState({kRootState}, std::make_unique<RuntimeState>("root")),
              "add root state");
    for (const HoverThrustRuntimeState state : kRuntimeStates) {
        requireOk(machine_->addState({stateId(state), kRootState},
                                     std::make_unique<RuntimeState>(stateName(state))),
                  "add runtime state");
    }
    requireOk(machine_->setInitialState(
                  {sm::kDefaultRegion, stateId(HoverThrustRuntimeState::kInitializing)}),
              "set initial state");
    for (const HoverThrustRuntimeState state : kRuntimeStates) {
        sm::TransitionRule transition;
        transition.from = kRootState;
        transition.target = stateId(state);
        transition.event = eventId(state);
        transition.global = true;
        requireOk(machine_->addTransition(transition), "add runtime transition");
    }
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
    const Input& input) const {
    Classification result;
    result.state = estimator_.valid() ? HoverThrustRuntimeState::kHoldingInputStale
                                      : HoverThrustRuntimeState::kInitializing;

    if (!input.imu_acc_z.received) {
        result.flags |= HoverThrustRuntimeFlag::kImuMissing;
        result.state = HoverThrustRuntimeState::kWaitingForImu;
        return result;
    }
    if (!input.normalized_thrust.received) {
        result.flags |= HoverThrustRuntimeFlag::kThrustMissing;
        result.state = HoverThrustRuntimeState::kWaitingForThrust;
        return result;
    }
    if (!input.altitude.received) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeMissing;
        result.state = HoverThrustRuntimeState::kWaitingForAltitude;
        return result;
    }

    if (sampleStale(input.imu_acc_z, input.now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kImuStale;
    }
    if (sampleStale(input.normalized_thrust, input.now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kThrustStale;
    }
    if (sampleStale(input.altitude, input.now_sec)) {
        result.flags |= HoverThrustRuntimeFlag::kAltitudeStale;
    }
    if (result.flags != 0) {
        result.state = HoverThrustRuntimeState::kHoldingInputStale;
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
    if ((result.flags & HoverThrustRuntimeFlag::kThrustInvalid) != 0) {
        result.state = HoverThrustRuntimeState::kInvalidThrust;
        return result;
    }
    if ((result.flags &
         (HoverThrustRuntimeFlag::kImuInvalid | HoverThrustRuntimeFlag::kAltitudeInvalid)) != 0) {
        result.state = (result.flags & HoverThrustRuntimeFlag::kImuInvalid) != 0
                           ? HoverThrustRuntimeState::kWaitingForImu
                           : HoverThrustRuntimeState::kWaitingForAltitude;
        return result;
    }
    if (input.altitude.value < config_.min_altitude) {
        result.flags |= HoverThrustRuntimeFlag::kBelowMinAltitude;
        result.state = HoverThrustRuntimeState::kBelowMinAltitude;
        return result;
    }

    result.ready = true;
    result.state = HoverThrustRuntimeState::kEstimating;
    result.source_stamp_sec = input.normalized_thrust.stamp_sec;
    return result;
}

bool HoverThrustEstimatorRuntime::sampleStale(const Sample& sample, double now_sec) const {
    if (!std::isfinite(now_sec) || !std::isfinite(sample.stamp_sec)) {
        return true;
    }
    if (sample.stamp_sec > now_sec + 0.05) {
        return true;
    }
    return now_sec - sample.stamp_sec > config_.sample_timeout;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::makeOutput(
    HoverThrustRuntimeState state, uint32_t flags, bool sample_used,
    double source_stamp_sec) const {
    Output output;
    output.state = state;
    output.flags = flags;
    output.state_name = stateName(state);
    output.hover_thrust = hover_thrust_output_;
    output.raw_hover_thrust = raw_hover_thrust_output_;
    output.initial_hover_thrust = config_.initial_hover_thrust;
    output.thrust_to_acceleration = output.hover_thrust > estimator_limits::kMinimumNormalizedThrust
                                        ? config_.gravity / output.hover_thrust
                                        : config_.gravity / config_.initial_hover_thrust;
    output.estimator_valid = estimator_.valid();
    output.sample_used = sample_used;
    output.source_stamp_sec = source_stamp_sec;
    output.last_estimate_stamp_sec = last_estimate_stamp_sec_;
    return output;
}

}  // namespace hover_thrust_estimator
