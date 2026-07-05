#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

#include <stdexcept>
#include <string>
#include <utility>

#include "hover_thrust_estimator/common/config_utils.h"
#include "hover_thrust_estimator/state_machine/airborne_state.h"
#include "hover_thrust_estimator/state_machine/ground_state.h"
#include "hover_thrust_estimator/state_machine/health_monitor_state.h"
#include "hover_thrust_estimator/state_machine/self_check_state.h"

namespace hover_thrust_estimator {
namespace {

namespace sm = state_machine;

void requireOk(const sm::Status& status, const char* operation) {
    if (!status.ok()) {
        throw std::runtime_error(std::string(operation) + ": " + status.message);
    }
}

}  // namespace

HoverThrustEstimatorRuntime::HoverThrustEstimatorRuntime() {
    config_utils::normalizeConfig(config_);
    reset();
}

void HoverThrustEstimatorRuntime::setConfig(const Config& config) {
    config_ = config;
    config_utils::normalizeConfig(config_);
    reset();
}

void HoverThrustEstimatorRuntime::reset() {
    estimator_.setConfig(HoverThrustEstimator::Config{config_.rho2, config_.min_hover_thrust,
                                                      config_.max_hover_thrust, false, 0.0});
    estimator_.reset(config_.gravity, config_.initial_hover_thrust);
    input_ = Input{};
    health_ = HealthStatus{};
    output_model_.reset(config_);
    current_time_sec_ = 0.0;
    last_estimate_stamp_sec_ = 0.0;
    last_sample_used_ = false;
    state_ = state_type::SelfCheck;
    flags_ = 0;
    last_output_ = makeOutput(state_, flags_, false, health_);
    debug_trace_records_.clear();
    setupMachine();
}

sm::Status HoverThrustEstimatorRuntime::postInputEvent(sm::Event event, const Input& input) {
    applyInputEvent(input);
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    debug_trace_records_.clear();
    const auto transition_result = machine_->update({64, 64, false});
    appendDebugTrace(HoverThrustDebugTracePhase::kTransitionPass, machine_->currentTrace());
    requireOk(transition_result.status, "update hover thrust estimator transitions");
    const auto tick_result = machine_->update({64, 64, true});
    appendDebugTrace(HoverThrustDebugTracePhase::kTickPass, machine_->currentTrace());
    requireOk(tick_result.status, "tick hover thrust estimator states");
    return last_output_;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output(double now_sec) const {
    (void)now_sec;
    return makeOutput(state_, flags_, false, health_);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::refreshOutputSnapshot() {
    last_output_ = makeOutput(state_, flags_, last_sample_used_, health_);
    return last_output_;
}

std::vector<sm::Event> HoverThrustEstimatorRuntime::debugOutputEvents(double now_sec) const {
    if (debug_trace_records_.empty()) {
        return {};
    }

    sm::Event event(output_event_type::PUBLISH_DEBUG_TRACE, sm::EventTimestamp{now_sec});
    event.category = sm::EventCategory::kOutput;
    event.source = "hover_thrust_estimator_runtime";
    return {event};
}

void HoverThrustEstimatorRuntime::enterState(HoverThrustStateId state) {
    state_ = state;
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
        .endRegion();

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ground)
        .on(event_type::HEALTH_BELOW_MIN_ALTITUDE)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Airborne)
        .on(event_type::HEALTH_ESTIMATION_READY)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Ground)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_INPUT_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Ground)
        .to(state_type::Airborne)
        .on(event_type::HEALTH_ESTIMATION_READY)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Airborne)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_INPUT_UNHEALTHY)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Airborne)
        .to(state_type::Ground)
        .on(event_type::HEALTH_BELOW_MIN_ALTITUDE)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    auto machine_result = builder.build();
    requireOk(machine_result.status, "build hover thrust estimator state machine");
    machine_ = std::move(machine_result.value);
    requireOk(machine_->start(), "start hover thrust estimator state machine");
}

void HoverThrustEstimatorRuntime::applyInputEvent(const Input& input) {
    input_ = input;
}

void HoverThrustEstimatorRuntime::appendDebugTrace(HoverThrustDebugTracePhase phase,
                                                   const std::vector<sm::EventTraceRecord>& trace) {
    for (const auto& record : trace) {
        if (isDebugTraceRecordInteresting(record)) {
            debug_trace_records_.push_back(DebugTraceRecord{phase, record});
        }
    }
}

bool HoverThrustEstimatorRuntime::isDebugTraceRecordInteresting(
    const sm::EventTraceRecord& record) {
    switch (record.kind) {
        case sm::EventTraceRecord::Kind::kInternalEventGenerated:
        case sm::EventTraceRecord::Kind::kTransitionCommitted:
        case sm::EventTraceRecord::Kind::kInternalEventDeferred:
            return true;
        case sm::EventTraceRecord::Kind::kEventConsumed:
            return record.event.category == sm::EventCategory::kInternal;
        case sm::EventTraceRecord::Kind::kOutputEventGenerated:
            return false;
    }
    return false;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::recordStateOutput(
    HoverThrustStateId state, uint32_t flags, bool sample_used) {
    flags_ = flags;
    state_ = state;
    last_sample_used_ = sample_used;
    last_output_ = makeOutput(state_, flags_, sample_used, health_);
    return last_output_;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::makeOutput(
    HoverThrustStateId state, uint32_t flags, bool sample_used, const HealthStatus& health) const {
    Output output;
    output.state = state;
    output.flags = flags;
    output.hover_thrust = output_model_.hoverThrust();
    output.raw_hover_thrust = output_model_.raw();
    output.initial_hover_thrust = config_.initial_hover_thrust;
    output.thrust_to_acceleration = output.hover_thrust > estimator_limits::kMinimumNormalizedThrust
                                        ? config_.gravity / output.hover_thrust
                                        : config_.gravity / config_.initial_hover_thrust;
    output.sample_used = sample_used;
    output.source_stamp_sec = health.source_stamp_sec;
    output.last_estimate_stamp_sec = last_estimate_stamp_sec_;
    return output;
}

}  // namespace hover_thrust_estimator
