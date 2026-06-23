#include "hover_thrust_estimator/state_machine/health_monitor_state.h"

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

HealthMonitorState::HealthMonitorState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext&) {
    runtime_.refreshHealth();
    return {};
}

}  // namespace hover_thrust_estimator
