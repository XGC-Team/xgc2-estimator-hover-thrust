#include "hover_thrust_estimator/state_machine/ground_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

GroundState::GroundState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::Ground), runtime_(runtime) {}

void GroundState::onEnter() {
    runtime_.enterState(HoverThrustRuntimeState::kGround);
}

void GroundState::onPerform() {
    runtime_.performGround();
    if (runtime_.consumePublishRequest()) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void GroundState::onExit() {}

}  // namespace hover_thrust_estimator
