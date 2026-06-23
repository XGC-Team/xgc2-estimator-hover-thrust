#include "hover_thrust_estimator/state_machine/airborne_state.h"

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

AirborneState::AirborneState(HoverThrustEstimatorRuntime& runtime) : runtime_(runtime) {}

::state_machine::ActionResult AirborneState::onEnter(::state_machine::StateContext& ctx) {
    (void)ctx;
    runtime_.enterState(state_type::Airborne);
    raw_update_gate_.reset();
    publish_gate_.reset();
    return {};
}

::state_machine::ActionResult AirborneState::onTick(::state_machine::StateContext& ctx) {
    if (runtime_.health().state != state_type::Airborne) {
        return {};
    }

    uint32_t flags = runtime_.health().flags;
    const bool sample_used = updateRawEstimateIfDue(flags);
    runtime_.recordStateOutput(state_type::Airborne, flags, sample_used);
    publishEstimateIfDue(ctx);
    return {};
}

bool AirborneState::updateRawEstimateIfDue(uint32_t& flags) {
    if (!raw_update_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().raw_update_rate_hz)) {
        return false;
    }
    if (!runtime_.health().ready) {
        flags |= HoverThrustRuntimeFlag::kRawEstimateStale;
        return false;
    }

    const auto& input = runtime_.input();
    const bool sample_used = runtime_.estimator().update(
        input.imu_acc_z.value, input.normalized_thrust.value, input.normalized_thrust.stamp_sec);
    if (!sample_used) {
        flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
        return false;
    }

    runtime_.outputModel().setRaw(runtime_.estimator().rawEstimate());
    runtime_.outputModel().setTarget(runtime_.estimator().rawEstimate());
    runtime_.setLastEstimateStamp(input.normalized_thrust.stamp_sec);
    return true;
}

void AirborneState::publishEstimateIfDue(::state_machine::StateContext& ctx) {
    if (publish_gate_.due(runtime_.currentTime(), 1.0 / runtime_.config().publish_rate_hz)) {
        ::state_machine::Event event(output_event_type::PUBLISH_ESTIMATE,
                                     ::state_machine::EventTimestamp{runtime_.currentTime()});
        event.category = ::state_machine::EventCategory::kOutput;
        ctx.emitOutput(std::move(event));
    }
}

::state_machine::ActionResult AirborneState::onExit(::state_machine::StateContext& ctx) {
    (void)ctx;
    raw_update_gate_.reset();
    publish_gate_.reset();
    return {};
}

}  // namespace hover_thrust_estimator
