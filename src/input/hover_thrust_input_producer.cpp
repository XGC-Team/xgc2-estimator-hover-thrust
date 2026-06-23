#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"

#include <cmath>
#include <utility>

#include "hover_thrust_estimator/common/event_types.h"

namespace hover_thrust_estimator {

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
    const double stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    updateSamplePeriod(runtime_input_.imu_acc_z, stamp_sec);
    runtime_input_.imu_acc_z.stamp_sec = stamp_sec;
    runtime_input_.imu_acc_z.received = true;
    runtime_input_.imu_acc_z.finite = std::isfinite(msg->linear_acceleration.z);
    runtime_input_.now_sec = stamp_sec;
    postInputEvent(event_type::INPUT_IMU_UPDATED, "imu", stamp_sec);
}

void HoverThrustInputProducer::targetAttitudeCallback(
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
    runtime_input_.now_sec = stamp_sec;
    postInputEvent(event_type::INPUT_THRUST_UPDATED, "target_attitude", stamp_sec);
}

void HoverThrustInputProducer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    runtime_input_.altitude.value = msg->pose.position.z;
    const double stamp_sec = messageStampOrNow(msg->header.stamp).toSec();
    updateSamplePeriod(runtime_input_.altitude, stamp_sec);
    runtime_input_.altitude.stamp_sec = stamp_sec;
    runtime_input_.altitude.received = true;
    runtime_input_.altitude.finite = std::isfinite(msg->pose.position.z);
    runtime_input_.now_sec = stamp_sec;
    postInputEvent(event_type::INPUT_ALTITUDE_UPDATED, "altitude", stamp_sec);
}

void HoverThrustInputProducer::postInputEvent(::state_machine::EventId event_id, const char* source,
                                              double timestamp_sec) {
    if (!event_sink_) {
        ROS_ERROR("[HoverThrustInputProducer] Event sink is not configured");
        return;
    }

    ::state_machine::Event event(event_id, ::state_machine::EventTimestamp{timestamp_sec});
    event.source = source;
    event.category = ::state_machine::EventCategory::kInput;
    const auto status = event_sink_(std::move(event), runtime_input_);
    if (!status.ok()) {
        ROS_ERROR_THROTTLE(1.0,
                           "[HoverThrustInputProducer] Failed to post input event %u from %s: %s",
                           event_id, source, status.message.c_str());
    }
}

void HoverThrustInputProducer::updateSamplePeriod(HoverThrustEstimatorRuntime::Sample& sample,
                                                  double stamp_sec) {
    sample.period_sec =
        sample.received && std::isfinite(sample.stamp_sec) && std::isfinite(stamp_sec)
            ? stamp_sec - sample.stamp_sec
            : 0.0;
}

ros::Time HoverThrustInputProducer::messageStampOrNow(const ros::Time& stamp) {
    return stamp.isZero() ? ros::Time::now() : stamp;
}

}  // namespace hover_thrust_estimator
