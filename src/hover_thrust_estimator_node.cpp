#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace hover_thrust_estimator {
namespace {

constexpr double kDefaultGravity = estimator_limits::kDefaultGravity;
constexpr double kDefaultInitialHoverThrust = 0.5;
constexpr double kDefaultRho2 = 0.998;
constexpr double kDefaultMinHoverThrust = 0.05;
constexpr double kDefaultMaxHoverThrust = 0.95;
constexpr double kDefaultMinAltitude = 0.5;
constexpr double kDefaultSampleTimeout = 0.2;
constexpr double kDefaultPublishRate = 100.0;
constexpr double kDefaultRawUpdateRate = 10.0;
constexpr double kDefaultFilterCutoffHz = 2.0;
constexpr double kMaxPublishRate = 1000.0;
constexpr double kDefaultInputRateLowHz = 5.0;
constexpr uint32_t kRosQueueSize = 10;

double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

bool rawUpdateDue(double now_sec, double last_update_sec, bool initialized,
                  double update_period_sec) {
    if (!std::isfinite(now_sec)) {
        return false;
    }
    if (!initialized) {
        return true;
    }
    if (now_sec + 0.05 < last_update_sec) {
        return true;
    }
    return now_sec - last_update_sec >= update_period_sec;
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
        "estimate=%s publish_rate=%.1f raw_update_rate=%.1f",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), estimate_topic_.c_str(), publish_rate_, raw_update_rate_);
}

HoverThrustEstimatorNode::~HoverThrustEstimatorNode() {
    output_event_executor_.stop();
}

void HoverThrustEstimatorNode::run(double frequency) {
    const double loop_frequency =
        std::isfinite(frequency) && frequency > 0.0 ? frequency : publish_rate_;
    const double raw_update_period = 1.0 / raw_update_rate_;
    bool raw_update_initialized = false;
    double last_raw_update_sec = 0.0;

    ROS_INFO("[HoverThrustEstimatorNode] Starting estimator loop at %.1f Hz", loop_frequency);

    ros::Rate rate(loop_frequency);
    while (ros::ok()) {
        ros::spinOnce();

        const double now_sec = ros::Time::now().toSec();
        if (rawUpdateDue(now_sec, last_raw_update_sec, raw_update_initialized, raw_update_period)) {
            runtime_.requestRawUpdate(now_sec);
            last_raw_update_sec = now_sec;
            raw_update_initialized = true;
        }

        runtime_.update(now_sec);
        dispatchOutputEvents(runtime_.getStateMachine().currentOutputEvents());
        rate.sleep();
    }

    ROS_INFO("[HoverThrustEstimatorNode] Estimator loop exited");
}

void HoverThrustEstimatorNode::loadParams() {
    private_nh_.param("imu_topic", imu_topic_, imu_topic_);
    private_nh_.param("target_attitude_topic", target_attitude_topic_, target_attitude_topic_);
    private_nh_.param("altitude_topic", altitude_topic_, altitude_topic_);
    private_nh_.param("estimate_state_topic", estimate_state_topic_, estimate_state_topic_);
    private_nh_.param("estimate_topic", estimate_topic_, estimate_topic_);

    private_nh_.param("gravity", gravity_, gravity_);
    private_nh_.param("initial_hover_thrust", initial_hover_thrust_, initial_hover_thrust_);
    private_nh_.param("rho2", rho2_, rho2_);
    private_nh_.param("min_hover_thrust", min_hover_thrust_, min_hover_thrust_);
    private_nh_.param("max_hover_thrust", max_hover_thrust_, max_hover_thrust_);
    private_nh_.param("min_altitude", min_altitude_, min_altitude_);
    private_nh_.param("sample_timeout", sample_timeout_, sample_timeout_);
    private_nh_.param("publish_rate", publish_rate_, publish_rate_);
    private_nh_.param("raw_update_rate", raw_update_rate_, raw_update_rate_);
    private_nh_.param("filter_enabled", filter_enabled_, filter_enabled_);
    private_nh_.param("filter_cutoff_hz", filter_cutoff_hz_, filter_cutoff_hz_);
    private_nh_.param("input_rate_low_hz", input_rate_low_hz_, input_rate_low_hz_);

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
    if (!std::isfinite(publish_rate_) || publish_rate_ <= 0.0) {
        publish_rate_ = kDefaultPublishRate;
    }
    publish_rate_ = std::min(publish_rate_, kMaxPublishRate);
    if (!std::isfinite(raw_update_rate_) || raw_update_rate_ <= 0.0) {
        raw_update_rate_ = kDefaultRawUpdateRate;
    }
    raw_update_rate_ = std::min(raw_update_rate_, kMaxPublishRate);
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
