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
    if (runtime_.health().state != HoverThrustRuntimeState::kAirborne) {
        return;
    }

    uint32_t flags = runtime_.health().flags;
    bool sample_used = false;

    if (runtime_.consumeRawUpdateRequest()) {
        if (runtime_.health().ready) {
            const auto& input = runtime_.input();
            sample_used =
                runtime_.estimator().update(input.imu_acc_z.value, input.normalized_thrust.value,
                                            input.normalized_thrust.stamp_sec);
            if (sample_used) {
                runtime_.setRawHoverThrust(runtime_.estimator().rawEstimate());
                runtime_.setTargetHoverThrust(runtime_.estimator().rawEstimate());
                runtime_.setLastEstimateStamp(input.normalized_thrust.stamp_sec);
            } else {
                flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
            }
        } else {
            flags |= HoverThrustRuntimeFlag::kRawEstimateStale;
        }
    }

    runtime_.driveOutputToward(runtime_.targetHoverThrust());
    runtime_.publishForState(HoverThrustRuntimeState::kAirborne, flags, sample_used);
    if (runtime_.consumePublishRequest()) {
        emitOutputEvent(output_event_type::PUBLISH_ESTIMATE, runtime_.currentTime());
    }
}

void AirborneState::onExit() {}

}  // namespace hover_thrust_estimator
