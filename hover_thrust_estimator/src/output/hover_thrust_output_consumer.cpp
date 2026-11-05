#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"

#include <ros1_utils/time_utils.h>

#include <cmath>
#include <stdexcept>
#include <string>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {
namespace {

uint8_t toMessageState(HoverThrustStateId state) {
    switch (state) {
        case state_type::SelfCheck:
            return hover_thrust_estimator_msgs::HoverThrustEstimate::STATE_SELF_CHECK;
        case state_type::Ground:
            return hover_thrust_estimator_msgs::HoverThrustEstimate::STATE_GROUND;
        case state_type::Airborne:
            return hover_thrust_estimator_msgs::HoverThrustEstimate::STATE_AIRBORNE;
        default:
            throw std::runtime_error("unknown hover thrust estimator state: " +
                                     std::to_string(state));
    }
}

}  // namespace

HoverThrustOutputConsumer::HoverThrustOutputConsumer(ros::NodeHandle& nh,
                                                     std::string estimate_state_topic,
                                                     uint32_t queue_size) {
    estimate_state_pub_ = nh.advertise<hover_thrust_estimator_msgs::HoverThrustEstimate>(
        estimate_state_topic, queue_size, true);
}

void HoverThrustOutputConsumer::handle(const PublishedEstimate& publication) {
    const ros::Time stamp = ros1_utils::timeSecOrNow(publication.stamp_sec);
    estimate_state_pub_.publish(makeEstimateStateMessage(publication.output, stamp));
}

hover_thrust_estimator_msgs::HoverThrustEstimate
HoverThrustOutputConsumer::makeEstimateStateMessage(const HoverThrustOutput& output,
                                                    const ros::Time& stamp) const {
    hover_thrust_estimator_msgs::HoverThrustEstimate msg;
    msg.header.stamp = stamp;
    msg.state = toMessageState(output.state);
    msg.flags = output.flags;
    msg.hover_thrust = output.hover_thrust;
    return msg;
}

}  // namespace hover_thrust_estimator
