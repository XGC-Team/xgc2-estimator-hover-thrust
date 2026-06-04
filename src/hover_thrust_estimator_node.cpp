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
constexpr double kDefaultPublishRate = 10.0;
constexpr double kDefaultFilterCutoffHz = 2.0;
constexpr double kMaxPublishRate = 1000.0;

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
        sample_timeout_, filter_enabled_, filter_cutoff_hz_});

    estimate_state_pub_ =
        nh_.advertise<hover_thrust_estimator::HoverThrustEstimate>(estimate_state_topic_, 10, true);
    estimate_pub_ = nh_.advertise<std_msgs::Float64>(estimate_topic_, 10, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 10, true);

    imu_sub_ = nh_.subscribe(imu_topic_, 20, &HoverThrustEstimatorNode::imuCallback, this);
    target_attitude_sub_ = nh_.subscribe(target_attitude_topic_, 20,
                                         &HoverThrustEstimatorNode::targetAttitudeCallback, this);
    pose_sub_ = nh_.subscribe(altitude_topic_, 10, &HoverThrustEstimatorNode::poseCallback, this);

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &HoverThrustEstimatorNode::updateCallback, this);

    const ros::Time startup_stamp = ros::Time::now();
    publishOutput(runtime_.output(), startup_stamp);

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s state=%s "
        "estimate=%s",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), estimate_topic_.c_str());
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
    private_nh_.param("filter_enabled", filter_enabled_, filter_enabled_);
    private_nh_.param("filter_cutoff_hz", filter_cutoff_hz_, filter_cutoff_hz_);

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
    if (!std::isfinite(filter_cutoff_hz_)) {
        filter_cutoff_hz_ = kDefaultFilterCutoffHz;
    }
    if (filter_cutoff_hz_ <= 0.0) {
        filter_enabled_ = false;
    }
}

void HoverThrustEstimatorNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.imu_acc_z.value = msg->linear_acceleration.z;
    runtime_input_.imu_acc_z.stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    runtime_input_.imu_acc_z.received = true;
    runtime_input_.imu_acc_z.finite = std::isfinite(msg->linear_acceleration.z);
}

void HoverThrustEstimatorNode::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    runtime_input_.thrust_ignored =
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0;
    runtime_input_.normalized_thrust.value = msg->thrust;
    runtime_input_.normalized_thrust.stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    runtime_input_.normalized_thrust.received = true;
    runtime_input_.normalized_thrust.finite = std::isfinite(msg->thrust);
}

void HoverThrustEstimatorNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.altitude.value = msg->pose.position.z;
    runtime_input_.altitude.stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    runtime_input_.altitude.received = true;
    runtime_input_.altitude.finite = std::isfinite(msg->pose.position.z);
}

void HoverThrustEstimatorNode::updateCallback(const ros::TimerEvent& event) {
    const ros::Time now = event.current_real.isZero() ? ros::Time::now() : event.current_real;
    runtime_input_.now_sec = now.toSec();
    publishOutput(runtime_.update(runtime_input_), now);
}

void HoverThrustEstimatorNode::publishOutput(const HoverThrustEstimatorRuntime::Output& output,
                                             const ros::Time& stamp) {
    hover_thrust_estimator::HoverThrustEstimate msg;
    msg.header.stamp = stamp;
    msg.state = static_cast<uint8_t>(output.state);
    msg.flags = output.flags;
    msg.state_name = output.state_name;
    msg.hover_thrust = output.hover_thrust;
    msg.raw_hover_thrust = output.raw_hover_thrust;
    msg.initial_hover_thrust = output.initial_hover_thrust;
    msg.thrust_to_acceleration = output.thrust_to_acceleration;
    msg.estimator_valid = output.estimator_valid;
    msg.sample_used = output.sample_used;
    msg.source_stamp = output.source_stamp_sec;
    msg.last_estimate_stamp = output.last_estimate_stamp_sec;
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

}  // namespace hover_thrust_estimator
