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
constexpr double kDefaultPublishRate = 50.0;
constexpr double kDefaultFilterCutoffHz = 2.0;
constexpr double kMaxPublishRate = 1000.0;
constexpr double kStampFutureToleranceSec = 0.05;

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

    estimator_.setConfig(HoverThrustEstimator::Config{rho2_, min_hover_thrust_, max_hover_thrust_,
                                                      filter_enabled_, filter_cutoff_hz_});
    estimator_.reset(gravity_, initial_hover_thrust_);

    estimate_pub_ = nh_.advertise<std_msgs::Float64>(estimate_topic_, 10, true);
    valid_pub_ = nh_.advertise<std_msgs::Bool>(valid_topic_, 10, true);

    imu_sub_ = nh_.subscribe(imu_topic_, 20, &HoverThrustEstimatorNode::imuCallback, this);
    target_attitude_sub_ = nh_.subscribe(target_attitude_topic_, 20,
                                         &HoverThrustEstimatorNode::targetAttitudeCallback, this);
    pose_sub_ = nh_.subscribe(altitude_topic_, 10, &HoverThrustEstimatorNode::poseCallback, this);

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_),
                                    &HoverThrustEstimatorNode::updateCallback, this);

    publishValid(false);

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s estimate=%s",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_topic_.c_str());
}

void HoverThrustEstimatorNode::loadParams() {
    private_nh_.param("imu_topic", imu_topic_, imu_topic_);
    private_nh_.param("target_attitude_topic", target_attitude_topic_, target_attitude_topic_);
    private_nh_.param("altitude_topic", altitude_topic_, altitude_topic_);
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
    if (!std::isfinite(msg->linear_acceleration.z)) {
        imu_sample_.received = false;
        return;
    }
    latest_acc_z_ = msg->linear_acceleration.z;
    imu_sample_.stamp = messageStampOrNow(msg->header.stamp);
    imu_sample_.received = true;
}

void HoverThrustEstimatorNode::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        thrust_valid_ = false;
        return;
    }

    thrust_valid_ = (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) == 0 &&
                    std::isfinite(msg->thrust) &&
                    msg->thrust > estimator_limits::kMinimumNormalizedThrust &&
                    msg->thrust <= estimator_limits::kMaximumNormalizedThrust;
    latest_thrust_ = msg->thrust;
    thrust_sample_.stamp = messageStampOrNow(msg->header.stamp);
    thrust_sample_.received = true;
}

void HoverThrustEstimatorNode::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    if (!std::isfinite(msg->pose.position.z)) {
        altitude_sample_.received = false;
        return;
    }
    latest_altitude_ = msg->pose.position.z;
    altitude_sample_.stamp = messageStampOrNow(msg->header.stamp);
    altitude_sample_.received = true;
}

void HoverThrustEstimatorNode::updateCallback(const ros::TimerEvent& event) {
    const ros::Time now = event.current_real.isZero() ? ros::Time::now() : event.current_real;
    const bool ready = sampleReady(now);
    if (ready) {
        estimator_.update(latest_acc_z_, latest_thrust_, thrust_sample_.stamp.toSec());
    }

    const bool valid = estimator_.valid() && ready;
    if (valid) {
        publishEstimate();
    }
    publishValid(valid);
}

bool HoverThrustEstimatorNode::sampleFresh(const TopicSample& sample, const ros::Time& now) const {
    return sample.received && sample.stamp <= now + ros::Duration(kStampFutureToleranceSec) &&
           (now - sample.stamp).toSec() <= sample_timeout_;
}

bool HoverThrustEstimatorNode::sampleReady(const ros::Time& now) const {
    if (!sampleFresh(imu_sample_, now) || !sampleFresh(thrust_sample_, now) ||
        !sampleFresh(altitude_sample_, now)) {
        return false;
    }
    if (!thrust_valid_ || !std::isfinite(latest_altitude_) || latest_altitude_ < min_altitude_) {
        return false;
    }
    return true;
}

void HoverThrustEstimatorNode::publishEstimate() {
    if (!estimator_.valid()) {
        return;
    }
    std_msgs::Float64 msg;
    msg.data = estimator_.estimate();
    estimate_pub_.publish(msg);
}

void HoverThrustEstimatorNode::publishValid(bool valid) {
    if (valid_state_published_ && valid == last_published_valid_) {
        return;
    }
    std_msgs::Bool msg;
    msg.data = valid;
    valid_pub_.publish(msg);
    valid_state_published_ = true;
    last_published_valid_ = valid;
}

}  // namespace hover_thrust_estimator
