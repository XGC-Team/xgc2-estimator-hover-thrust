#include "hover_thrust_estimator/state_machine/fault_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

FaultState::FaultState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::Fault), runtime_(runtime) {}

void FaultState::onEnter() {
    runtime_.enterState(state_type::Fault);
}

void FaultState::onPerform() {
    (void)runtime_.consumeRawUpdateRequest();
    runtime_.publishForState(state_type::Fault,
                             runtime_.health().flags | HoverThrustRuntimeFlag::kStateMachineFault,
                             false);
    if (runtime_.consumePublishRequest()) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void FaultState::onExit() {}

}  // namespace hover_thrust_estimator
