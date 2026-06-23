#include "hover_thrust_estimator/state_machine/airborne_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

AirborneState::AirborneState(HoverThrustEstimatorRuntime& runtime)
    : StateAdapter(state_type::Airborne), runtime_(runtime) {}

void AirborneState::onEnter() {
    runtime_.enterState(HoverThrustRuntimeState::kAirborne);
}

void AirborneState::onPerform() {
    runtime_.performAirborne();
    emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
}

void AirborneState::onExit() {}

}  // namespace hover_thrust_estimator
