#include "hover_thrust_estimator/state_machine/fault_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

FaultState::FaultState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult FaultState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Fault);
    publish_gate_.reset();
    return {};
}

::state_machine::ActionResult FaultState::onTick(::state_machine::StateContext& ctx) {
    runtime_.recordStateOutput(state_type::Fault,
                               runtime_.health().flags | HoverThrustRuntimeFlag::kStateMachineFault,
                               false);
    publishEstimateIfDue(ctx);
    return {};
}

void FaultState::publishEstimateIfDue(::state_machine::StateContext& ctx) {
    if (publish_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_ESTIMATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult FaultState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    publish_gate_.reset();
    return {};
}

}  // namespace hover_thrust_estimator
