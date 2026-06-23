#include "hover_thrust_estimator/state_machine/health_monitor_state.h"

#include <utility>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/health_checks.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

double eventTimeOrCurrent(const ::state_machine::Event& event,
                          const HoverThrustEstimatorRuntime& runtime) {
    return event.timestamp > 0.0 ? event.timestamp : runtime.currentTime();
}

}  // namespace

HealthMonitorState::HealthMonitorState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onEvent(::state_machine::StateContext& ctx,
                                                          const ::state_machine::Event& event) {
    evaluateAndPostTransition(ctx, eventTimeOrCurrent(event, runtime_));
    return {};
}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext&) {
    runtime_.setHealth(health_checks::classify(runtime_.input(), runtime_.config(),
                                               runtime_.faultRequested(), runtime_.currentTime()));
    return {};
}

void HealthMonitorState::evaluateAndPostTransition(::state_machine::StateContext& ctx,
                                                   double now_sec) const {
    const auto health = health_checks::classify(runtime_.input(), runtime_.config(),
                                                runtime_.faultRequested(), now_sec);
    runtime_.setHealth(health);

    const ::state_machine::StateId active_state = ctx.currentState(region_type::ESTIMATION);
    if (active_state == health.state) {
        return;
    }

    if (health.transition_event == 0) {
        return;
    }

    ::state_machine::Event event(health.transition_event, ::state_machine::EventTimestamp{now_sec});
    event.source = "health_monitor_state";
    event.category = ::state_machine::EventCategory::kInternal;
    const auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        runtime_.setHealth(
            HoverThrustHealthStatus{state_type::Fault, event_type::HEALTH_TO_FAULT,
                                    health.flags | HoverThrustRuntimeFlag::kStateMachineFault,
                                    false, health.source_stamp_sec});
    }
}

}  // namespace hover_thrust_estimator
