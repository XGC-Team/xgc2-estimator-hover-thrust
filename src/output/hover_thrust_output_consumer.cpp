#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"

#include <memory>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {
namespace {

ros::Time eventStampOrNow(const ::state_machine::Event& event) {
    return event.timestamp > 0.0 ? ros::Time(event.timestamp) : ros::Time::now();
}

std::unique_ptr<RosOutputTask> makePublishTask(
    std::string name, ros::Publisher estimate_state_pub, ros::Publisher estimate_pub,
    hover_thrust_estimator::HoverThrustEstimate estimate_state_msg,
    std_msgs::Float64 estimate_msg) {
    return std::make_unique<RosLambdaOutputTask>(
        std::move(name),
        [estimate_state_pub = std::move(estimate_state_pub), estimate_pub = std::move(estimate_pub),
         estimate_state_msg = std::move(estimate_state_msg),
         estimate_msg = std::move(estimate_msg)](ros::NodeHandle&) mutable {
            estimate_state_pub.publish(estimate_state_msg);
            estimate_pub.publish(estimate_msg);
        });
}

}  // namespace

HoverThrustOutputConsumer::HoverThrustOutputConsumer(
    ros::NodeHandle& nh, RosOutputExecutor& executor, HoverThrustEstimatorRuntime& runtime,
    std::string estimate_state_topic, std::string estimate_topic, uint32_t queue_size)
    : executor_(executor), runtime_(runtime) {
    estimate_state_pub_ = nh.advertise<hover_thrust_estimator::HoverThrustEstimate>(
        estimate_state_topic, queue_size, true);
    estimate_pub_ = nh.advertise<std_msgs::Float64>(estimate_topic, queue_size, true);
}

bool HoverThrustOutputConsumer::handle(const ::state_machine::Event& event) {
    if (event.id != output_event_type::PUBLISH_ESTIMATE) {
        return false;
    }

    const HoverThrustEstimatorRuntime::Output output = runtime_.snapshotOutput();
    const ros::Time stamp = eventStampOrNow(event);
    executor_.pushTask(makePublishTask("PublishHoverThrustEstimate", estimate_state_pub_,
                                       estimate_pub_, makeEstimateStateMessage(output, stamp),
                                       makeEstimateMessage(output)));
    return true;
}

hover_thrust_estimator::HoverThrustEstimate HoverThrustOutputConsumer::makeEstimateStateMessage(
    const HoverThrustEstimatorRuntime::Output& output, const ros::Time& stamp) const {
    hover_thrust_estimator::HoverThrustEstimate msg;
    msg.header.stamp = stamp;
    msg.state = static_cast<uint8_t>(output.state);
    msg.flags = output.flags;
    msg.hover_thrust = output.hover_thrust;
    return msg;
}

std_msgs::Float64 HoverThrustOutputConsumer::makeEstimateMessage(
    const HoverThrustEstimatorRuntime::Output& output) {
    std_msgs::Float64 msg;
    msg.data = output.hover_thrust;
    return msg;
}

}  // namespace hover_thrust_estimator
