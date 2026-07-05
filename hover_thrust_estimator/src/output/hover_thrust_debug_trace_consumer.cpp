#include "hover_thrust_estimator/output/hover_thrust_debug_trace_consumer.h"

#include <ros1_utils/time_utils.h>
#include <state_machine_msgs/StateMachineTraceEvent.h>

#include <memory>
#include <string>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

uint8_t toMessagePhase(HoverThrustDebugTracePhase phase) {
    switch (phase) {
        case HoverThrustDebugTracePhase::kTransitionPass:
            return state_machine_msgs::StateMachineTraceEvent::PHASE_TRANSITION;
        case HoverThrustDebugTracePhase::kTickPass:
            return state_machine_msgs::StateMachineTraceEvent::PHASE_TICK;
    }
    return 0;
}

uint8_t toMessageKind(::state_machine::EventTraceRecord::Kind kind) {
    switch (kind) {
        case ::state_machine::EventTraceRecord::Kind::kInternalEventGenerated:
            return state_machine_msgs::StateMachineTraceEvent::KIND_INTERNAL_EVENT_GENERATED;
        case ::state_machine::EventTraceRecord::Kind::kOutputEventGenerated:
            return state_machine_msgs::StateMachineTraceEvent::KIND_OUTPUT_EVENT_GENERATED;
        case ::state_machine::EventTraceRecord::Kind::kEventConsumed:
            return state_machine_msgs::StateMachineTraceEvent::KIND_EVENT_CONSUMED;
        case ::state_machine::EventTraceRecord::Kind::kTransitionCommitted:
            return state_machine_msgs::StateMachineTraceEvent::KIND_TRANSITION_COMMITTED;
        case ::state_machine::EventTraceRecord::Kind::kInternalEventDeferred:
            return state_machine_msgs::StateMachineTraceEvent::KIND_INTERNAL_EVENT_DEFERRED;
    }
    return 0;
}

uint8_t toMessageCategory(::state_machine::EventCategory category) {
    switch (category) {
        case ::state_machine::EventCategory::kInput:
            return state_machine_msgs::StateMachineTraceEvent::CATEGORY_INPUT;
        case ::state_machine::EventCategory::kInternal:
            return state_machine_msgs::StateMachineTraceEvent::CATEGORY_INTERNAL;
        case ::state_machine::EventCategory::kOutput:
            return state_machine_msgs::StateMachineTraceEvent::CATEGORY_OUTPUT;
    }
    return 0;
}

std::string eventName(::state_machine::EventId event_id) {
    switch (event_id) {
        case event_type::INPUT_IMU_UPDATED:
            return "INPUT_IMU_UPDATED";
        case event_type::INPUT_THRUST_UPDATED:
            return "INPUT_THRUST_UPDATED";
        case event_type::INPUT_ALTITUDE_UPDATED:
            return "INPUT_ALTITUDE_UPDATED";
        case event_type::HEALTH_INPUT_UNHEALTHY:
            return "HEALTH_INPUT_UNHEALTHY";
        case event_type::HEALTH_BELOW_MIN_ALTITUDE:
            return "HEALTH_BELOW_MIN_ALTITUDE";
        case event_type::HEALTH_ESTIMATION_READY:
            return "HEALTH_ESTIMATION_READY";
        case output_event_type::PUBLISH_ESTIMATE:
            return "PUBLISH_ESTIMATE";
        case output_event_type::PUBLISH_DEBUG_TRACE:
            return "PUBLISH_DEBUG_TRACE";
        default:
            return "UNKNOWN";
    }
}

state_machine_msgs::StateMachineTraceEvent makeTraceEventMessage(
    const HoverThrustDebugTraceRecord& record) {
    state_machine_msgs::StateMachineTraceEvent msg;
    const auto& trace = record.trace;
    msg.phase = toMessagePhase(record.phase);
    msg.kind = toMessageKind(trace.kind);
    msg.event_id = trace.event.id;
    msg.event_name = eventName(trace.event.id);
    msg.category = toMessageCategory(trace.event.category);
    msg.source = trace.event.source;
    msg.sequence = trace.event.sequence;
    msg.correlation_id = trace.event.correlation_id;
    msg.producer_region = trace.producer_region;
    msg.producer_state = trace.producer_state;
    msg.consumer_region = trace.consumer_region;
    msg.from_state = trace.from_state;
    msg.to_state = trace.to_state;
    msg.transition_id = trace.transition.value_or(0);
    msg.priority = trace.priority;
    return msg;
}

std::unique_ptr<::state_machine::runtime::Task<ros::NodeHandle>> makePublishTask(
    std::string name, ros::Publisher debug_trace_pub,
    state_machine_msgs::StateMachineTrace debug_trace_msg) {
    return std::make_unique<::state_machine::runtime::LambdaTask<ros::NodeHandle>>(
        std::move(name), [debug_trace_pub = std::move(debug_trace_pub),
                          debug_trace_msg = std::move(debug_trace_msg)](ros::NodeHandle&) mutable {
            debug_trace_pub.publish(debug_trace_msg);
        });
}

}  // namespace

HoverThrustDebugTraceConsumer::HoverThrustDebugTraceConsumer(
    ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
    HoverThrustEstimatorRuntime& runtime, std::string debug_trace_topic, uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    debug_trace_pub_ =
        nh.advertise<state_machine_msgs::StateMachineTrace>(debug_trace_topic, queue_size, false);
}

bool HoverThrustDebugTraceConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_DEBUG_TRACE) {
        return false;
    }
    if (!runtime_.hasDebugTrace()) {
        return true;
    }

    const ros::Time stamp = ros1_utils::timeSecOrNow(event.timestamp);
    executor_.pushTask(makePublishTask("PublishHoverThrustStateMachineTrace", debug_trace_pub_,
                                       makeTraceMessage(stamp)));
    return true;
}

state_machine_msgs::StateMachineTrace HoverThrustDebugTraceConsumer::makeTraceMessage(
    const ros::Time& stamp) const {
    state_machine_msgs::StateMachineTrace msg;
    msg.header.stamp = stamp;
    const auto& machine = runtime_.getStateMachine();
    msg.machine_name = machine.name();
    const auto snapshot = machine.snapshot();
    msg.update_index = snapshot.update_index;
    msg.active_region_ids.reserve(snapshot.active_leaf_states.size());
    msg.active_state_ids.reserve(snapshot.active_leaf_states.size());
    for (const auto& entry : snapshot.active_leaf_states) {
        msg.active_region_ids.push_back(entry.first);
        msg.active_state_ids.push_back(entry.second);
    }

    const auto& records = runtime_.debugTraceRecords();
    msg.events.reserve(records.size());
    for (const auto& record : records) {
        msg.events.push_back(makeTraceEventMessage(record));
    }
    return msg;
}

}  // namespace hover_thrust_estimator
