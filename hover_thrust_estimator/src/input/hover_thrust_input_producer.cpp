#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"

#include <ros1_utils/time_utils.h>

#include <cmath>
#include <state_machine/runtime/event_post.hpp>
#include <string>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {
namespace {

void logInputEventPostFailure(const ::state_machine::Status& status,
                              ::state_machine::EventId failed_event_id,
                              const std::string& failed_source) {
    ROS_WARN_THROTTLE(1.0, "[HoverThrustInputProducer] Failed to post input event %u from %s: %s",
                      failed_event_id, failed_source.c_str(), status.message.c_str());
}

}  // namespace

HoverThrustInputProducer::HoverThrustInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                                                   std::string target_attitude_topic,
                                                   std::string altitude_topic, uint32_t queue_size,
                                                   EventSink event_sink)
    : event_sink_(std::move(event_sink)) {
    imu_sub_ = nh.subscribe(std::move(imu_topic), queue_size,
                            &HoverThrustInputProducer::imuCallback, this);
    target_attitude_sub_ = nh.subscribe(std::move(target_attitude_topic), queue_size,
                                        &HoverThrustInputProducer::targetAttitudeCallback, this);
    pose_sub_ = nh.subscribe(std::move(altitude_topic), queue_size,
                             &HoverThrustInputProducer::poseCallback, this);
}

void HoverThrustInputProducer::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.imu_acc_z.value = msg->linear_acceleration.z;
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    ros1_utils::updateSamplePeriod(runtime_input_.imu_acc_z, stamp_sec);
    runtime_input_.imu_acc_z.stamp_sec = stamp_sec;
    runtime_input_.imu_acc_z.received = true;
    runtime_input_.imu_acc_z.finite = std::isfinite(msg->linear_acceleration.z);
    (void)::state_machine::runtime::postInputEventWithFailureHandler(
        event_sink_, event_type::INPUT_IMU_UPDATED, "imu", stamp_sec, logInputEventPostFailure,
        runtime_input_);
}

void HoverThrustInputProducer::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    runtime_input_.thrust_ignored =
        (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0;
    runtime_input_.normalized_thrust.value = msg->thrust;
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    ros1_utils::updateSamplePeriod(runtime_input_.normalized_thrust, stamp_sec);
    runtime_input_.normalized_thrust.stamp_sec = stamp_sec;
    runtime_input_.normalized_thrust.received = true;
    runtime_input_.normalized_thrust.finite = std::isfinite(msg->thrust);
    (void)::state_machine::runtime::postInputEventWithFailureHandler(
        event_sink_, event_type::INPUT_THRUST_UPDATED, "target_attitude", stamp_sec,
        logInputEventPostFailure, runtime_input_);
}

void HoverThrustInputProducer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.altitude.value = msg->pose.position.z;
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    ros1_utils::updateSamplePeriod(runtime_input_.altitude, stamp_sec);
    runtime_input_.altitude.stamp_sec = stamp_sec;
    runtime_input_.altitude.received = true;
    runtime_input_.altitude.finite = std::isfinite(msg->pose.position.z);
    (void)::state_machine::runtime::postInputEventWithFailureHandler(
        event_sink_, event_type::INPUT_ALTITUDE_UPDATED, "altitude", stamp_sec,
        logInputEventPostFailure, runtime_input_);
}

}  // namespace hover_thrust_estimator
