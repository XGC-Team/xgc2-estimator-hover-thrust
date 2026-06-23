#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <ros1_utils/param_utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "hover_thrust_estimator/common/config_utils.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kDefaultLoopRate = 1000.0;
constexpr double kDefaultPublishRate = 100.0;
constexpr double kDefaultRawUpdateRate = 10.0;
constexpr double kMaxLoopRate = 5000.0;
constexpr uint32_t kRosQueueSize = 10;

}  // namespace

HoverThrustEstimatorNode::HoverThrustEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();

    runtime_.setConfig(HoverThrustEstimatorRuntime::Config{
        gravity_, initial_hover_thrust_, rho2_, min_hover_thrust_, max_hover_thrust_, min_altitude_,
        sample_timeout_, filter_enabled_, filter_cutoff_hz_, input_rate_low_hz_, publish_rate_,
        raw_update_rate_});

    output_event_dispatcher_.addConsumer(std::make_unique<HoverThrustOutputConsumer>(
        nh_, output_event_executor_, runtime_, estimate_state_topic_, estimate_topic_,
        kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event, const HoverThrustInput& input) {
        return runtime_.postInputEvent(std::move(event), input);
    };

    input_producer_ = std::make_unique<HoverThrustInputProducer>(
        nh_, imu_topic_, target_attitude_topic_, altitude_topic_, kRosQueueSize, post_input_event);

    output_event_executor_.start();

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s state=%s "
        "estimate=%s loop_rate=%.1f publish_rate=%.1f raw_update_rate=%.1f",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), estimate_topic_.c_str(), loop_rate_, publish_rate_,
        raw_update_rate_);
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
        loop_frequency, publish_rate_, raw_update_rate_);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();

        const double now_sec = ros::Time::now().toSec();
        runtime_.update(now_sec);
        dispatchOutputEvents(runtime_.getStateMachine().currentOutputEvents());
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
    ros1_utils::getParamWithLog(private_nh_, "estimate_topic", estimate_topic_,
                                "Legacy hover thrust estimate topic");

    ros1_utils::getParamWithLog(private_nh_, "gravity", gravity_, "Gravity");
    ros1_utils::getParamWithLog(private_nh_, "initial_hover_thrust", initial_hover_thrust_,
                                "Initial hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "rho2", rho2_, "RLS forgetting factor");
    ros1_utils::getParamWithLog(private_nh_, "min_hover_thrust", min_hover_thrust_,
                                "Minimum hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "max_hover_thrust", max_hover_thrust_,
                                "Maximum hover thrust");
    ros1_utils::getParamWithLog(private_nh_, "min_altitude", min_altitude_,
                                "Minimum estimation altitude");
    ros1_utils::getParamWithLog(private_nh_, "sample_timeout", sample_timeout_,
                                "Input sample timeout");
    ros1_utils::getParamWithLog(private_nh_, "loop_rate", loop_rate_, "Estimator loop rate");
    ros1_utils::getParamWithLog(private_nh_, "publish_rate", publish_rate_, "Publish rate");
    ros1_utils::getParamWithLog(private_nh_, "raw_update_rate", raw_update_rate_,
                                "RLS update rate");
    ros1_utils::getParamWithLog(private_nh_, "filter_enabled", filter_enabled_,
                                "Hover thrust low-pass filter");
    ros1_utils::getParamWithLog(private_nh_, "filter_cutoff_hz", filter_cutoff_hz_,
                                "Hover thrust low-pass cutoff");
    ros1_utils::getParamWithLog(private_nh_, "input_rate_low_hz", input_rate_low_hz_,
                                "Minimum healthy input rate");

    const auto estimator_config = config_utils::normalizeConfig(HoverThrustEstimatorConfig{
        gravity_, initial_hover_thrust_, rho2_, min_hover_thrust_, max_hover_thrust_, min_altitude_,
        sample_timeout_, filter_enabled_, filter_cutoff_hz_, input_rate_low_hz_, publish_rate_,
        raw_update_rate_});
    gravity_ = estimator_config.gravity;
    initial_hover_thrust_ = estimator_config.initial_hover_thrust;
    rho2_ = estimator_config.rho2;
    min_hover_thrust_ = estimator_config.min_hover_thrust;
    max_hover_thrust_ = estimator_config.max_hover_thrust;
    min_altitude_ = estimator_config.min_altitude;
    sample_timeout_ = estimator_config.sample_timeout;
    filter_enabled_ = estimator_config.filter_enabled;
    filter_cutoff_hz_ = estimator_config.filter_cutoff_hz;
    input_rate_low_hz_ = estimator_config.input_rate_low_hz;
    publish_rate_ = estimator_config.publish_rate_hz;
    raw_update_rate_ = estimator_config.raw_update_rate_hz;

    if (!std::isfinite(loop_rate_) || loop_rate_ <= 0.0) {
        loop_rate_ = kDefaultLoopRate;
    }
    loop_rate_ = std::min(loop_rate_, kMaxLoopRate);
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
        publish_rate_ = kDefaultPublishRate;
    }
    publish_rate_ = std::min(publish_rate_, loop_rate_);
    if (!std::isfinite(raw_update_rate_) || raw_update_rate_ <= 0.0) {
        raw_update_rate_ = kDefaultRawUpdateRate;
    }
    raw_update_rate_ = std::min(raw_update_rate_, loop_rate_);
}

void HoverThrustEstimatorNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    output_event_dispatcher_.dispatch(events);
}

}  // namespace hover_thrust_estimator
