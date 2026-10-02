#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <string>

#include "hover_thrust_estimator/common/types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustInputProducer {
   public:
    HoverThrustInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                             std::string target_attitude_topic, std::string altitude_topic,
                             uint32_t queue_size, HoverThrustEstimatorRuntime& runtime);

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void targetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    HoverThrustEstimatorRuntime& runtime_;
    ros::Subscriber imu_sub_;
    ros::Subscriber target_attitude_sub_;
    ros::Subscriber pose_sub_;
};

}  // namespace hover_thrust_estimator
