#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <functional>
#include <state_machine/state_machine.hpp>
#include <string>

#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator {

class HoverThrustInputProducer {
   public:
    using EventSink =
        std::function<::state_machine::Status(::state_machine::Event, const HoverThrustInput&)>;

    HoverThrustInputProducer(ros::NodeHandle& nh, std::string imu_topic,
                             std::string target_attitude_topic, std::string altitude_topic,
                             uint32_t queue_size, EventSink event_sink);

   private:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void targetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void postInputEvent(::state_machine::EventId event_id, const char* source,
                        double timestamp_sec);
    static void updateSamplePeriod(HoverThrustSample& sample, double stamp_sec);
    static ros::Time messageStampOrNow(const ros::Time& stamp);

    EventSink event_sink_;
    HoverThrustInput runtime_input_;
    ros::Subscriber imu_sub_;
    ros::Subscriber target_attitude_sub_;
    ros::Subscriber pose_sub_;
};

}  // namespace hover_thrust_estimator
