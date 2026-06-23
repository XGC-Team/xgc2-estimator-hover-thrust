#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <ros1_utils/param_utils.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace hover_thrust_estimator {
namespace {

constexpr double kDefaultGravity = estimator_limits::kDefaultGravity;
constexpr double kDefaultInitialHoverThrust = 0.3;
constexpr double kDefaultRho2 = 0.998;
constexpr double kDefaultMinHoverThrust = 0.15;
constexpr double kDefaultMaxHoverThrust = 0.85;
constexpr double kDefaultMinAltitude = 0.5;
constexpr double kDefaultSampleTimeout = 0.2;
constexpr double kDefaultLoopRate = 1000.0;
constexpr double kDefaultPublishRate = 100.0;
constexpr double kDefaultRawUpdateRate = 10.0;
constexpr double kDefaultFilterCutoffHz = 2.0;
constexpr double kMaxLoopRate = 5000.0;
constexpr double kDefaultInputRateLowHz = 5.0;
constexpr uint32_t kRosQueueSize = 10;

double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

bool timerDue(double now_sec, double last_update_sec, bool initialized, double period_sec) {
    if (!std::isfinite(now_sec)) {
        return false;
    }
    if (!initialized) {
        return true;
    }
    if (now_sec + 0.05 < last_update_sec) {
        return true;
    }
    return now_sec - last_update_sec >= period_sec;
}

}  // namespace

HoverThrustEstimatorNode::HoverThrustEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();

    runtime_.setConfig(HoverThrustEstimatorRuntime::Config{
        gravity_, initial_hover_thrust_, rho2_, min_hover_thrust_, max_hover_thrust_, min_altitude_,
        sample_timeout_, filter_enabled_, filter_cutoff_hz_, input_rate_low_hz_});

    output_event_dispatcher_.addConsumer(std::make_unique<HoverThrustOutputConsumer>(
        nh_, output_event_executor_, runtime_, estimate_state_topic_, estimate_topic_,
        kRosQueueSize));

    auto post_input_event = [this](::state_machine::Event event,
                                   const HoverThrustEstimatorRuntime::Input& input) {
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
    const double publish_period = 1.0 / publish_rate_;
    const double raw_update_period = 1.0 / raw_update_rate_;
    bool publish_initialized = false;
    bool raw_update_initialized = false;
    double last_publish_sec = 0.0;
    double last_raw_update_sec = 0.0;

    ROS_INFO(
        "[HoverThrustEstimatorNode] Starting estimator loop at %.1f Hz, publishing at %.1f Hz, "
        "updating RLS at %.1f Hz",
        loop_frequency, publish_rate_, raw_update_rate_);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();

        const double now_sec = ros::Time::now().toSec();
        if (timerDue(now_sec, last_raw_update_sec, raw_update_initialized, raw_update_period)) {
            runtime_.requestRawUpdate(now_sec);
            last_raw_update_sec = now_sec;
            raw_update_initialized = true;
        }
        if (timerDue(now_sec, last_publish_sec, publish_initialized, publish_period)) {
            runtime_.requestPublish(now_sec);
            last_publish_sec = now_sec;
            publish_initialized = true;
        }

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

    gravity_ = finiteOrDefault(gravity_, kDefaultGravity);
    if (gravity_ <= estimator_limits::kMinimumGravity) {
        gravity_ = kDefaultGravity;
    }
    min_hover_thrust_ = std::clamp(finiteOrDefault(min_hover_thrust_, kDefaultMinHoverThrust), 0.0,
                                   estimator_limits::kMaximumNormalizedThrust);
    max_hover_thrust_ = std::clamp(finiteOrDefault(max_hover_thrust_, kDefaultMaxHoverThrust),
                                   min_hover_thrust_, estimator_limits::kMaximumNormalizedThrust);
    initial_hover_thrust_ =
        std::clamp(finiteOrDefault(initial_hover_thrust_, kDefaultInitialHoverThrust),
                   min_hover_thrust_, max_hover_thrust_);
    if (!std::isfinite(rho2_) || rho2_ <= 0.0 || rho2_ > 1.0) {
        rho2_ = kDefaultRho2;
    }
    if (!std::isfinite(min_altitude_)) {
        min_altitude_ = kDefaultMinAltitude;
    }
    if (!std::isfinite(sample_timeout_) || sample_timeout_ <= 0.0) {
        sample_timeout_ = kDefaultSampleTimeout;
    }
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
    if (!std::isfinite(filter_cutoff_hz_)) {
        filter_cutoff_hz_ = kDefaultFilterCutoffHz;
    }
    if (filter_cutoff_hz_ <= 0.0) {
        filter_enabled_ = false;
    }
    if (!std::isfinite(input_rate_low_hz_) || input_rate_low_hz_ < 0.0) {
        input_rate_low_hz_ = kDefaultInputRateLowHz;
    }
}

void HoverThrustEstimatorNode::dispatchOutputEvents(
    const std::vector<::state_machine::Event>& events) {
    output_event_dispatcher_.dispatch(events);
}

}  // namespace hover_thrust_estimator
