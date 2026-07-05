#include "hover_thrust_estimator/state_machine/health_monitor_state.h"

#include <state_machine/runtime/event_time.hpp>
#include <stdexcept>
#include <string>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/health_checks.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

::state_machine::EventId eventForCondition(HoverThrustHealthCondition condition) {
    switch (condition) {
        case HoverThrustHealthCondition::kInputUnhealthy:
            return event_type::HEALTH_INPUT_UNHEALTHY;
        case HoverThrustHealthCondition::kBelowMinAltitude:
            return event_type::HEALTH_BELOW_MIN_ALTITUDE;
        case HoverThrustHealthCondition::kEstimationReady:
            return event_type::HEALTH_ESTIMATION_READY;
    }
    throw std::runtime_error("unknown hover thrust health condition: " +
                             std::to_string(static_cast<int>(condition)));
}

}  // namespace

HealthMonitorState::HealthMonitorState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult HealthMonitorState::onEvent(::state_machine::StateContext& ctx,
                                                          const ::state_machine::Event& event) {
    if (event.category != ::state_machine::EventCategory::kInput) {
        return {};
    }
    evaluateAndPostHealthEvent(
        ctx, ::state_machine::runtime::eventTimestampOr(event, runtime_.currentTime()));
    return {};
}

::state_machine::ActionResult HealthMonitorState::onTick(::state_machine::StateContext& ctx) {
    evaluateAndPostHealthEvent(ctx, runtime_.currentTime());
    return {};
}

void HealthMonitorState::evaluateAndPostHealthEvent(::state_machine::StateContext& ctx,
                                                    double now_sec) const {
    const HoverThrustHealthCondition previous_condition = runtime_.health().condition;
    const auto health = health_checks::classify(runtime_.input(), runtime_.config(), now_sec);
    runtime_.setHealth(health);

    if (health.condition == previous_condition) {
        return;
    }

    ::state_machine::Event event(eventForCondition(health.condition),
                                 ::state_machine::EventTimestamp{now_sec});
    event.source = "health_monitor_state";
    event.category = ::state_machine::EventCategory::kInternal;
    const auto status = ctx.postInternalEvent(std::move(event));
    if (!status.ok()) {
        throw std::runtime_error("post hover thrust health transition event: " + status.message);
    }
}

}  // namespace hover_thrust_estimator
