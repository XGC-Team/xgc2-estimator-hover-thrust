#include "hover_thrust_estimator/state_machine/self_check_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

SelfCheckState::SelfCheckState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::SelfCheck), runtime_(runtime) {}

void SelfCheckState::onEnter() {
    runtime_.enterState(HoverThrustRuntimeState::kSelfCheck);
}

void SelfCheckState::onPerform() {
    runtime_.performSelfCheck();
    emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
}

void SelfCheckState::onExit() {}

}  // namespace hover_thrust_estimator
