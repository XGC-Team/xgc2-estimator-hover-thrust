#include "hover_thrust_estimator/state_machine/fault_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

FaultState::FaultState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::Fault), runtime_(runtime) {}

void FaultState::onEnter() {
    runtime_.enterState(state_type::Fault);
    publish_gate_.reset();
}

void FaultState::onPerform() {
    runtime_.recordStateOutput(state_type::Fault,
                               runtime_.health().flags | HoverThrustRuntimeFlag::kStateMachineFault,
                               false);
    publishEstimateIfDue();
}

void FaultState::publishEstimateIfDue() {
    if (publish_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().publish_rate_hz)) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void FaultState::onExit() {
    publish_gate_.reset();
}

}  // namespace hover_thrust_estimator
