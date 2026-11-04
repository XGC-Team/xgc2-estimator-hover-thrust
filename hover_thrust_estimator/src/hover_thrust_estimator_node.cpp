#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

#include <ros/callback_queue.h>
#include <ros/timer.h>
#include <ros1_utils/param_utils.h>

#include <cmath>
#include <limits>
#include <memory>
#include <utility>

#include "hover_thrust_estimator/common/config_utils.h"

namespace hover_thrust_estimator {
namespace {

constexpr uint32_t kRosQueueSize = 10;

}  // namespace

HoverThrustEstimatorNode::HoverThrustEstimatorNode(ros::NodeHandle& nh)
    : nh_(nh), private_nh_("~"), output_event_executor_(nh_) {
    loadParams();

    runtime_.setConfig(estimator_config_);

    output_consumer_ = std::make_unique<HoverThrustOutputConsumer>(
        nh_, output_event_executor_, runtime_, estimate_state_topic_, kRosQueueSize);
    debug_trace_consumer_ = std::make_unique<HoverThrustDebugTraceConsumer>(
        nh_, output_event_executor_, runtime_, debug_trace_topic_, kRosQueueSize);

    input_producer_ = std::make_unique<HoverThrustInputProducer>(
        nh_, imu_topic_, target_attitude_topic_, altitude_topic_, kRosQueueSize, runtime_);

    deadline_timer_ = nh_.createTimer(ros::Duration(1.0),
                                      &HoverThrustEstimatorNode::onDeadlineTimer, this,
                                      /*oneshot=*/true, /*autostart=*/false);

    output_event_executor_.start();

    ROS_INFO(
        "[HoverThrustEstimatorNode] Initialized: imu=%s target_attitude=%s pose=%s estimate=%s "
        "debug_trace=%s publish_rate=%.1f raw_update_rate=%.1f",
        imu_topic_.c_str(), target_attitude_topic_.c_str(), altitude_topic_.c_str(),
        estimate_state_topic_.c_str(), debug_trace_topic_.c_str(),
        estimator_config_.publish_rate_hz, estimator_config_.raw_update_rate_hz);
}

HoverThrustEstimatorNode::~HoverThrustEstimatorNode() {
    deadline_timer_.stop();
    output_event_executor_.stop();
}

void HoverThrustEstimatorNode::run(double frequency) {
    (void)frequency;
    ROS_INFO(
        "[HoverThrustEstimatorNode] Starting event/deadline owner loop, publishing at %.1f Hz, "
        "updating RLS at %.1f Hz",
        estimator_config_.publish_rate_hz, estimator_config_.raw_update_rate_hz);

    // This is a finite no-wake safety bound for the callback queue wait, not
    // the ROS deadline and not a periodic poll. The one-shot ROS timer is the
    // deadline primitive; ros::shutdown disables the global queue and wakes
    // this wait.
    const ros::WallDuration owner_wait(std::numeric_limits<int32_t>::max(), 999999999);

    while (ros::ok()) {
        const double now_sec = ros::Time::now().toSec();
        const auto wake = runtime_.nextWake(now_sec);
        // Every active alternative owns a publish gate; nextDue() returns now
        // when that gate is uninitialized, so this is always populated.
        const double deadline_sec = *wake.next_deadline_sec;

        if (runtime_.hasPendingInput() || now_sec >= deadline_sec) {
            const double update_sec = ros::Time::now().toSec();
            runtime_.update(update_sec);
            publishRuntimeOutputs();
            continue;
        }

        const double arm_sec = ros::Time::now().toSec();
        if (arm_sec >= deadline_sec) {
            continue;
        }
        ros::Duration until(deadline_sec - arm_sec);
        if (until <= ros::Duration(0.0)) {
            // Preserve a strictly positive residual that rounded to zero.
            until = ros::Duration(0, 1);
        }

        deadline_timer_.stop();
        deadline_timer_.setPeriod(until, true);
        deadline_timer_.start();

        // The ROS clock may advance while the timer is being armed. Recheck
        // after arming and correct the already-started timer, without adding a
        // second scheduler or a fallback wake.
        const double after_arm_sec = ros::Time::now().toSec();
        if (after_arm_sec >= deadline_sec) {
            deadline_timer_.stop();
            continue;
        }
        if (after_arm_sec > arm_sec) {
            const double remaining_sec = deadline_sec - after_arm_sec;
            ros::Duration adjusted(remaining_sec);
            if (adjusted <= ros::Duration(0.0)) {
                adjusted = ros::Duration(0, 1);
            }
            deadline_timer_.setPeriod(adjusted, true);
        }

        ros::getGlobalCallbackQueue()->callAvailable(owner_wait);
        deadline_timer_.stop();
    }

    ROS_INFO("[HoverThrustEstimatorNode] Estimator loop exited");
}

void HoverThrustEstimatorNode::onDeadlineTimer(const ros::TimerEvent& event) {
    (void)event;
}

void HoverThrustEstimatorNode::publishRuntimeOutputs() {
    for (const auto& publication : runtime_.publishedEstimates()) {
        if (!output_consumer_->handle(publication.event)) {
            ROS_WARN_THROTTLE(1.0,
                              "[HoverThrustEstimatorNode] Published estimate was not found by its "
                              "consumer");
        }
    }

    const auto& transitions = runtime_.transitionTrace();
    if (!transitions.empty()) {
        debug_trace_consumer_->handle(transitions);
    }
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

}  // namespace hover_thrust_estimator
