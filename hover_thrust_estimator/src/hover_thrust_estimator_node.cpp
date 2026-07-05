#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <ros1_utils/param_utils.h>

#include <cmath>
#include <memory>
#include <utility>

#include "hover_thrust_estimator/common/config_utils.h"

namespace hover_thrust_estimator {
namespace {

constexpr uint32_t kRosQueueSize = 10;

void logOutputDispatchResult(const ::state_machine::runtime::EventDispatchResult& result) {
    for (const auto& event : result.unhandled_events) {
        ROS_WARN("[HoverThrustEstimatorNode] Unhandled output event id: %u",
                 static_cast<unsigned>(event.id));
    }
    for (const auto& failure : result.failures) {
        ROS_WARN("[HoverThrustEstimatorNode] Output consumer '%s' failed on event %u: %s",
                 failure.consumer_name.c_str(), static_cast<unsigned>(failure.event.id),
                 failure.message.c_str());
    }
}

}  // namespace

HoverThrustEstimatorNode::HoverThrustEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();

    runtime_.setConfig(estimator_config_);

    output_event_dispatcher_.addConsumer(std::make_unique<HoverThrustOutputConsumer>(
        nh_, output_event_executor_, runtime_, estimate_state_topic_, kRosQueueSize));
    output_event_dispatcher_.addConsumer(std::make_unique<HoverThrustDebugTraceConsumer>(
        nh_, output_event_executor_, runtime_, debug_trace_topic_, kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event, const HoverThrustInput& input) {
        return runtime_.postInputEvent(std::move(event), input);
    };

    input_producer_ = std::make_unique<HoverThrustInputProducer>(
        nh_, imu_topic_, target_attitude_topic_, altitude_topic_, kRosQueueSize, post_input_event);

    output_event_executor_.start();

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s estimate=%s "
        "debug_trace=%s loop_rate=%.1f publish_rate=%.1f raw_update_rate=%.1f",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), debug_trace_topic_.c_str(), loop_rate_,
        estimator_config_.publish_rate_hz, estimator_config_.raw_update_rate_hz);
}

HoverThrustEstimatorNode::~HoverThrustEstimatorNode() {
    output_event_executor_.stop();
}

void HoverThrustEstimatorNode::run(double frequency) {
    const double loop_frequency =
        std::isfinite(frequency) && frequency > 0.0 ? frequency : loop_rate_;

    ROS_INFO(
        "[HoverThrustEstimatorNode] Starting estimator loop at %.1f Hz, publishing at %.1f Hz, "
        "updating RLS at %.1f Hz",
        loop_frequency, estimator_config_.publish_rate_hz, estimator_config_.raw_update_rate_hz);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();

        const double now_sec = ros::Time::now().toSec();
        runtime_.update(now_sec);
        dispatchOutputEvents(runtime_.getStateMachine().currentOutputEvents());
        dispatchOutputEvents(runtime_.debugOutputEvents(now_sec));
        rate.sleep();
    }

    ROS_INFO("[HoverThrustEstimatorNode] Estimator loop exited");
}

void HoverThrustEstimatorNode::loadParams() {
    ros1_utils::getParamWithLog(private_nh_, "imu_topic", imu_topic_, "IMU topic");
    ros1_utils::getParamWithLog(private_nh_, "target_attitude_topic", target_attitude_topic_,
                                "Target attitude topic");
    ros1_utils::getParamWithLog(private_nh_, "altitude_topic", altitude_topic_, "Altitude topic");
    ros1_utils::getParamWithLog(private_nh_, "estimate_state_topic", estimate_state_topic_,
                                "Hover thrust estimate state topic");
    ros1_utils::getParamWithLog(private_nh_, "debug_trace_topic", debug_trace_topic_,
                                "State-machine debug trace topic");

    ros1_utils::getParamWithLog(private_nh_, "gravity", estimator_config_.gravity, "Gravity");
    ros1_utils::getParamWithLog(private_nh_, "initial_hover_thrust",
                                estimator_config_.initial_hover_thrust, "Initial hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "rho2", estimator_config_.rho2,
                                "RLS forgetting factor");
    ros1_utils::getParamWithLog(private_nh_, "min_hover_thrust", estimator_config_.min_hover_thrust,
                                "Minimum hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "max_hover_thrust", estimator_config_.max_hover_thrust,
                                "Maximum hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "min_altitude", estimator_config_.min_altitude,
                                "Minimum estimation altitude");
    ros1_utils::getParamWithLog(private_nh_, "sample_timeout", estimator_config_.sample_timeout,
                                "Input sample timeout");
    ros1_utils::getParamWithLog(private_nh_, "loop_rate", loop_rate_, "Estimator loop rate");
    ros1_utils::getParamWithLog(private_nh_, "publish_rate", estimator_config_.publish_rate_hz,
                                "Publish rate");
    ros1_utils::getParamWithLog(private_nh_, "raw_update_rate",
                                estimator_config_.raw_update_rate_hz, "RLS update rate");
    ros1_utils::getParamWithLog(private_nh_, "filter_enabled", estimator_config_.filter_enabled,
                                "Hover thrust low-pass filter");
    ros1_utils::getParamWithLog(private_nh_, "filter_cutoff_hz", estimator_config_.filter_cutoff_hz,
                                "Hover thrust low-pass cutoff");
    ros1_utils::getParamWithLog(private_nh_, "input_rate_low_hz",
                                estimator_config_.input_rate_low_hz, "Minimum healthy input rate");

    config_utils::normalizeLoopAndEstimatorRates(loop_rate_, estimator_config_);
}

void HoverThrustEstimatorNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    const auto result = output_event_dispatcher_.dispatch(events);
    logOutputDispatchResult(result);
}

}  // namespace hover_thrust_estimator
