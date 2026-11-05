#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"

#include <ros1_utils/time_utils.h>

#include <cmath>
#include <string>
#include <utility>

namespace hover_thrust_estimator {

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
    runtime_.ingestImu(msg->linear_acceleration.z, stamp_sec, stamp_sec);
}

void HoverThrustInputProducer::targetAttitudeCallback(
    const mavros_msgs::AttitudeTarget::ConstPtr& msg) {
    if (!msg) {
        return;
    }

    const bool ignored = (msg->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) != 0;
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    runtime_.ingestThrust(msg->thrust, ignored, stamp_sec, stamp_sec);
}

void HoverThrustInputProducer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!msg) {
        return;
    }
    const double stamp_sec = ros1_utils::messageStampOrNow(msg->header.stamp).toSec();
    runtime_.ingestAltitude(msg->pose.position.z, stamp_sec, stamp_sec);
}

}  // namespace hover_thrust_estimator
