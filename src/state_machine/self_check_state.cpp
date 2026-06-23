#include "hover_thrust_estimator/state_machine/self_check_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

SelfCheckState::SelfCheckState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::SelfCheck), runtime_(runtime) {}

void SelfCheckState::onEnter() {
    runtime_.enterState(state_type::SelfCheck);
}

void SelfCheckState::onPerform() {
    if (runtime_.health().state != state_type::SelfCheck) {
        return;
    }

    runtime_.outputModel().setTarget(runtime_.config().initial_hover_thrust);
    runtime_.outputModel().driveTowardTarget(runtime_.currentTime());
    runtime_.publishForState(state_type::SelfCheck, runtime_.health().flags, false);
    if (runtime_.consumePublishRequest()) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void SelfCheckState::onExit() {}

}  // namespace hover_thrust_estimator
