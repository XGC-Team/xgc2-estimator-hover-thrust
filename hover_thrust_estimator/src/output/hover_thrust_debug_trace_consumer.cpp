#include "hover_thrust_estimator/output/hover_thrust_debug_trace_consumer.h"

#include <ros1_utils/time_utils.h>
#include <state_machine_msgs/StateMachineTraceEvent.h>

#include <string>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {
namespace {

std::string eventName(uint32_t event_id) {
    switch (event_id) {
        case event_type::HEALTH_INPUT_UNHEALTHY:
            return "HEALTH_INPUT_UNHEALTHY";
        case event_type::HEALTH_BELOW_MIN_ALTITUDE:
            return "HEALTH_BELOW_MIN_ALTITUDE";
        case event_type::HEALTH_ESTIMATION_READY:
            return "HEALTH_ESTIMATION_READY";
        default:
            return "UNKNOWN";
    }
}

state_machine_msgs::StateMachineTraceEvent makeTraceEventMessage(const TransitionTrace& trace) {
    state_machine_msgs::StateMachineTraceEvent msg;
    msg.phase = state_machine_msgs::StateMachineTraceEvent::PHASE_TRANSITION;
    msg.kind = state_machine_msgs::StateMachineTraceEvent::KIND_TRANSITION_COMMITTED;
    msg.event_id = trace.event_id;
    msg.event_name = eventName(trace.event_id);
    msg.category = state_machine_msgs::StateMachineTraceEvent::CATEGORY_INTERNAL;
    msg.source = "hover_thrust.owner";
    msg.sequence = trace.sequence;
    msg.correlation_id = 0;
    msg.producer_region = region_type::ESTIMATION;
    msg.producer_state = trace.from_state;
    msg.consumer_region = region_type::ESTIMATION;
    msg.from_state = trace.from_state;
    msg.to_state = trace.to_state;
    msg.transition_id = trace.sequence;
    msg.priority = trace.priority;
    return msg;
}

}  // namespace

HoverThrustDebugTraceConsumer::HoverThrustDebugTraceConsumer(ros::NodeHandle& nh,
                                                             HoverThrustEstimatorRuntime& runtime,
                                                             std::string debug_trace_topic,
                                                             uint32_t queue_size)
    : runtime_(runtime) {
    debug_trace_pub_ =
        nh.advertise<state_machine_msgs::StateMachineTrace>(debug_trace_topic, queue_size, false);
}

bool HoverThrustDebugTraceConsumer::handle(const std::vector<TransitionTrace>& traces) {
    if (traces.empty()) {
        return false;
    }
    const ros::Time stamp = ros1_utils::timeSecOrNow(traces.back().stamp_sec);
    debug_trace_pub_.publish(makeTraceMessage(stamp, traces));
    return true;
}

state_machine_msgs::StateMachineTrace HoverThrustDebugTraceConsumer::makeTraceMessage(
    const ros::Time& stamp, const std::vector<TransitionTrace>& traces) const {
    state_machine_msgs::StateMachineTrace msg;
    msg.header.stamp = stamp;
    msg.machine_name = "HoverThrustEstimator";
    msg.update_index = runtime_.updateIndex();
    msg.active_region_ids = {region_type::ESTIMATION};
    msg.active_state_ids = {runtime_.currentState()};

    msg.events.reserve(traces.size());
    for (const auto& trace : traces) {
        msg.events.push_back(makeTraceEventMessage(trace));
    }
    return msg;
}

}  // namespace hover_thrust_estimator
