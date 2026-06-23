#include "hover_thrust_estimator/state_machine/ground_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

GroundState::GroundState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::Ground), runtime_(runtime) {}

void GroundState::onEnter() {
    runtime_.enterState(state_type::Ground);
    runtime_.outputModel().freezeTarget();
    publish_gate_.reset();
}

void GroundState::onPerform() {
    if (runtime_.health().state != state_type::Ground) {
        return;
    }

    runtime_.recordStateOutput(state_type::Ground, runtime_.health().flags, false);
    publishEstimateIfDue();
}

void GroundState::publishEstimateIfDue() {
    if (publish_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().publish_rate_hz)) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void GroundState::onExit() {
    publish_gate_.reset();
}

}  // namespace hover_thrust_estimator
