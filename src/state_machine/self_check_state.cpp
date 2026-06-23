#include "hover_thrust_estimator/state_machine/self_check_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

SelfCheckState::SelfCheckState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::SelfCheck), runtime_(runtime) {}

void SelfCheckState::onEnter() {
    runtime_.enterState(state_type::SelfCheck);
    runtime_.outputModel().setTarget(runtime_.config().initial_hover_thrust);
    publish_gate_.reset();
}

void SelfCheckState::onPerform() {
    if (runtime_.health().state != state_type::SelfCheck) {
        return;
    }

    runtime_.recordStateOutput(state_type::SelfCheck, runtime_.health().flags, false);
    publishEstimateIfDue();
}

void SelfCheckState::publishEstimateIfDue() {
    if (publish_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().publish_rate_hz)) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void SelfCheckState::onExit() {
    publish_gate_.reset();
}

}  // namespace hover_thrust_estimator
