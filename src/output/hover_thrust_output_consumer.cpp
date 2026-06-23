#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"

#include <memory>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

ros::Time eventStampOrNow(const ::state_machine::Event& event) {
    return event.timestamp > 0.0 ? ros::Time(event.timestamp) : ros::Time::now();
}

uint8_t toMessageState(::state_machine::StateId state) {
    switch (state) {
        case state_type::SelfCheck:
            return hover_thrust_estimator::HoverThrustEstimate::STATE_SELF_CHECK;
        case state_type::Ground:
            return hover_thrust_estimator::HoverThrustEstimate::STATE_GROUND;
        case state_type::Airborne:
            return hover_thrust_estimator::HoverThrustEstimate::STATE_AIRBORNE;
        case state_type::Fault:
            return hover_thrust_estimator::HoverThrustEstimate::STATE_FAULT;
        default:
            return hover_thrust_estimator::HoverThrustEstimate::STATE_FAULT;
    }
}

std::unique_ptr<RosOutputTask> makePublishTask(
    std::string name, ros::Publisher estimate_state_pub,
    hover_thrust_estimator::HoverThrustEstimate estimate_state_msg) {
    return std::make_unique<RosLambdaOutputTask>(
        std::move(name),
        [estimate_state_pub = std::move(estimate_state_pub),
         estimate_state_msg = std::move(estimate_state_msg)](ros::NodeHandle&) mutable {
            estimate_state_pub.publish(estimate_state_msg);
        });
}

}  // namespace

HoverThrustOutputConsumer::HoverThrustOutputConsumer(ros::NodeHandle& nh,
                                                     RosOutputExecutor& executor,
                                                     HoverThrustEstimatorRuntime& runtime,
                                                     std::string estimate_state_topic,
                                                     uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    estimate_state_pub_ = nh.advertise<hover_thrust_estimator::HoverThrustEstimate>(
        estimate_state_topic, queue_size, true);
}

bool HoverThrustOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_ESTIMATE) {
        return false;
    }

    const ros::Time stamp = eventStampOrNow(event);
    runtime_.outputModel().driveTowardTarget(stamp.toSec());
    const HoverThrustOutput output = runtime_.refreshOutputSnapshot();
    executor_.pushTask(makePublishTask("PublishHoverThrustEstimate", estimate_state_pub_,
                                       makeEstimateStateMessage(output, stamp)));
    return true;
}

hover_thrust_estimator::HoverThrustEstimate HoverThrustOutputConsumer::makeEstimateStateMessage(
    const HoverThrustOutput& output, const ros::Time& stamp) const {
    hover_thrust_estimator::HoverThrustEstimate msg;
    msg.header.stamp = stamp;
    msg.state = toMessageState(output.state);
    msg.flags = output.flags;
    msg.hover_thrust = output.hover_thrust;
    return msg;
}

}  // namespace hover_thrust_estimator
