#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

#include <stdexcept>
#include <string>
#include <utility>

#include "hover_thrust_estimator/common/config_utils.h"
#include "hover_thrust_estimator/state_machine/airborne_state.h"
#include "hover_thrust_estimator/state_machine/fault_state.h"
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
    config_ = config_utils::normalizeConfig(config_);
    reset();
}

void HoverThrustEstimatorRuntime::setConfig(const Config& config) {
    config_ = config_utils::normalizeConfig(config);
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
    raw_update_requested_ = false;
    publish_requested_ = false;
    fault_requested_ = false;
    state_ = state_type::SelfCheck;
    flags_ = 0;
    last_output_ = makeOutput(state_, flags_, false, health_);
    setupMachine();
}

sm::Status HoverThrustEstimatorRuntime::postInputEvent(sm::Event event, const Input& input) {
    applyInputEvent(input);
    event.category = sm::EventCategory::kInput;
    return machine_->postEvent(std::move(event));
}

void HoverThrustEstimatorRuntime::requestRawUpdate(double now_sec) {
    current_time_sec_ = now_sec;
    raw_update_requested_ = true;
    sm::Event event(event_type::INPUT_RAW_UPDATE_DUE, sm::EventTimestamp{now_sec});
    event.source = "raw_update_timer";
    event.category = sm::EventCategory::kInput;
    requireOk(machine_->postEvent(std::move(event)), "post raw update event");
}

void HoverThrustEstimatorRuntime::requestPublish(double now_sec) {
    current_time_sec_ = now_sec;
    publish_requested_ = true;
    sm::Event event(event_type::INPUT_PUBLISH_DUE, sm::EventTimestamp{now_sec});
    event.source = "publish_timer";
    event.category = sm::EventCategory::kInput;
    requireOk(machine_->postEvent(std::move(event)), "post publish event");
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(double now_sec) {
    current_time_sec_ = now_sec;
    const auto transition_result = machine_->update({64, 64, false});
    const auto tick_result =
        transition_result.status.ok() ? machine_->update({64, 64, true}) : transition_result;
    if (!tick_result.status.ok()) {
        fault_requested_ = true;
        state_ = state_type::Fault;
        flags_ |= HoverThrustRuntimeFlag::kStateMachineFault;
        last_output_ = makeOutput(state_, flags_, false, health_);
    }
    return last_output_;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output(double now_sec) const {
    (void)now_sec;
    return makeOutput(state_, flags_, false, health_);
}

void HoverThrustEstimatorRuntime::enterState(HoverThrustStateId state) {
    state_ = state;
}

bool HoverThrustEstimatorRuntime::consumeRawUpdateRequest() {
    if (!raw_update_requested_) {
        return false;
    }
    raw_update_requested_ = false;
    return true;
}

bool HoverThrustEstimatorRuntime::consumePublishRequest() {
    if (!publish_requested_) {
        return false;
    }
    publish_requested_ = false;
    return true;
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

    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Ground)
        .on(event_type::HEALTH_TO_GROUND)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Airborne)
        .on(event_type::HEALTH_TO_AIRBORNE)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::SelfCheck)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Ground)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Ground)
        .to(state_type::Airborne)
        .on(event_type::HEALTH_TO_AIRBORNE)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Ground)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Airborne)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Airborne)
        .to(state_type::Ground)
        .on(event_type::HEALTH_TO_GROUND)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Airborne)
        .to(state_type::Fault)
        .on(event_type::HEALTH_TO_FAULT)
        .priority(transition_priority::FAULT)
        .evaluationOrder(0);

    builder.transition()
        .from(state_type::Fault)
        .to(state_type::SelfCheck)
        .on(event_type::HEALTH_TO_SELF_CHECK)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Fault)
        .to(state_type::Ground)
        .on(event_type::HEALTH_TO_GROUND)
        .priority(transition_priority::AUTOMATIC)
        .evaluationOrder(0);
    builder.transition()
        .from(state_type::Fault)
        .to(state_type::Airborne)
        .on(event_type::HEALTH_TO_AIRBORNE)
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

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::publishForState(
    HoverThrustStateId state, uint32_t flags, bool sample_used) {
    if (state == state_type::Fault) {
        flags |= HoverThrustRuntimeFlag::kStateMachineFault;
    }
    flags_ = flags;
    state_ = state;
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
