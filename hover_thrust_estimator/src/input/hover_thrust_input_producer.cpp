#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"

#include <ros1_utils/time_utils.h>

#include <cmath>
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
                                                   HoverThrustEstimatorRuntime& runtime)
    : runtime_(runtime) {
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
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    const auto status = runtime_.ingestImu(msg->linear_acceleration.z, stamp_sec, stamp_sec);
    if (!status.ok())
        logInputEventPostFailure(status, event_type::INPUT_IMU_UPDATED, "imu");
}

void HoverThrustInputProducer::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const bool ignored = (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0;
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    const auto status = runtime_.ingestThrust(msg->thrust, ignored, stamp_sec, stamp_sec);
    if (!status.ok())
        logInputEventPostFailure(status, event_type::INPUT_THRUST_UPDATED, "target_attitude");
}

void HoverThrustInputProducer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    const auto status = runtime_.ingestAltitude(msg->pose.position.z, stamp_sec, stamp_sec);
    if (!status.ok())
        logInputEventPostFailure(status, event_type::INPUT_ALTITUDE_UPDATED, "altitude");
}

}  // namespace hover_thrust_estimator
