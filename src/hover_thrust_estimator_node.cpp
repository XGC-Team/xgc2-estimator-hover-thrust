#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <algorithm>
#include <cmath>

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
constexpr double kDefaultOutputSlewRate = 0.1;
constexpr double kDefaultInputRateLowHz = 5.0;

double finiteOrDefault(double value, double fallback) {
    return std::isfinite(value) ? value : fallback;
}

ros::Time messageStampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now() : stamp;
}

}  // namespace

HoverThrustEstimatorNode::HoverThrustEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~") {
    loadParams();

    runtime_.setConfig(HoverThrustEstimatorRuntime::Config{
        gravity_, initial_hover_thrust_, rho2_, min_hover_thrust_, max_hover_thrust_, min_altitude_,
        sample_timeout_, filter_enabled_, filter_cutoff_hz_, output_slew_rate_,
        input_rate_low_hz_});

    estimate_state_pub_ =
        nh_.advertise<hover_thrust_estimator::HoverThrustEstimate>(estimate_state_topic_, 10, true);
    estimate_pub_ = nh_.advertise<std_msgs::Float64>(estimate_topic_, 10, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 10, true);

    imu_sub_ = nh_.subscribe(imu_topic_, 20, &HoverThrustEstimatorNode::imuCallback, this);
    target_attitude_sub_ = nh_.subscribe(target_attitude_topic_, 20,
                                         &HoverThrustEstimatorNode::targetAttitudeCallback, this);
    pose_sub_ = nh_.subscribe(altitude_topic_, 10, &HoverThrustEstimatorNode::poseCallback, this);

    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                     &HoverThrustEstimatorNode::publishTimerCallback, this);
    raw_update_timer_ = nh_.createTimer(ros::Duration(1.0 / raw_update_rate_),
                                        &HoverThrustEstimatorNode::rawUpdateTimerCallback, this);

    const ros::Time startup_stamp = ros::Time::now();
    publishOutput(runtime_.output(startup_stamp.toSec()), startup_stamp);

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s state=%s "
        "estimate=%s publish_rate=%.1f raw_update_rate=%.1f",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), estimate_topic_.c_str(), publish_rate_, raw_update_rate_);
}

void HoverThrustEstimatorNode::loadParams() {
    private_nh_.param("imu_topic", imu_topic_, imu_topic_);
    private_nh_.param("target_attitude_topic", target_attitude_topic_, target_attitude_topic_);
    private_nh_.param("altitude_topic", altitude_topic_, altitude_topic_);
    private_nh_.param("estimate_state_topic", estimate_state_topic_, estimate_state_topic_);
    private_nh_.param("estimate_topic", estimate_topic_, estimate_topic_);
    private_nh_.param("valid_topic", valid_topic_, valid_topic_);

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
    private_nh_.param("output_slew_rate", output_slew_rate_, output_slew_rate_);
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
    if (!std::isfinite(output_slew_rate_) || output_slew_rate_ < 0.0) {
        output_slew_rate_ = kDefaultOutputSlewRate;
    }
    if (!std::isfinite(input_rate_low_hz_) || input_rate_low_hz_ < 0.0) {
        input_rate_low_hz_ = kDefaultInputRateLowHz;
    }
}

void HoverThrustEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.imu_acc_z.value = msg->linear_acceleration.z;
    const double stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    updateSamplePeriod(runtime_input_.imu_acc_z, stamp_sec);
    runtime_input_.imu_acc_z.stamp_sec = stamp_sec;
    runtime_input_.imu_acc_z.received = true;
    runtime_input_.imu_acc_z.finite = std::isfinite(msg->linear_acceleration.z);
    postInputEvent(HoverThrustInputEvent::kImuUpdated);
}

void HoverThrustEstimatorNode::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    runtime_input_.thrust_ignored =
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0;
    runtime_input_.normalized_thrust.value = msg->thrust;
    const double stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    updateSamplePeriod(runtime_input_.normalized_thrust, stamp_sec);
    runtime_input_.normalized_thrust.stamp_sec = stamp_sec;
    runtime_input_.normalized_thrust.received = true;
    runtime_input_.normalized_thrust.finite = std::isfinite(msg->thrust);
    postInputEvent(HoverThrustInputEvent::kThrustUpdated);
}

void HoverThrustEstimatorNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.altitude.value = msg->pose.position.z;
    const double stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    updateSamplePeriod(runtime_input_.altitude, stamp_sec);
    runtime_input_.altitude.stamp_sec = stamp_sec;
    runtime_input_.altitude.received = true;
    runtime_input_.altitude.finite = std::isfinite(msg->pose.position.z);
    postInputEvent(HoverThrustInputEvent::kAltitudeUpdated);
}

void HoverThrustEstimatorNode::publishTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    publishOutput(runtime_.update(HoverThrustOutputEvent::kPublishEstimate, now.toSec()), now);
}

void HoverThrustEstimatorNode::rawUpdateTimerCallback(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    publishOutput(runtime_.update(HoverThrustOutputEvent::kUpdateRawEstimate, now.toSec()), now);
}

void HoverThrustEstimatorNode::publishOutput(const HoverThrustEstimatorRuntime::Output& output,
                                             const ros::Time& stamp) {
    hover_thrust_estimator::HoverThrustEstimate msg;
    msg.header.stamp = stamp;
    msg.state = static_cast<uint8_t>(output.state);
    msg.flags = output.flags;
    msg.hover_thrust = output.hover_thrust;
    msg.estimate_valid = output.estimate_valid;
    estimate_state_pub_.publish(msg);

    publishEstimate(output.hover_thrust);
    publishValid(output.estimator_valid);
}

void HoverThrustEstimatorNode::publishEstimate(double hover_thrust) {
    std_msgs::Float64 msg;
    msg.data = hover_thrust;
    estimate_pub_.publish(msg);
}

void HoverThrustEstimatorNode::publishValid(bool valid) {
    std_msgs::Bool msg;
    msg.data = valid;
    valid_pub_.publish(msg);
}

void HoverThrustEstimatorNode::postInputEvent(HoverThrustInputEvent event) {
    runtime_.postInputEvent(event, runtime_input_);
}

void HoverThrustEstimatorNode::updateSamplePeriod(HoverThrustEstimatorRuntime::Sample& sample,
                                                  double stamp_sec) {
    sample.period_sec =
        sample.received && std::isfinite(sample.stamp_sec) && std::isfinite(stamp_sec)
            ? stamp_sec - sample.stamp_sec
            : 0.0;
}

}  // namespace hover_thrust_estimator
