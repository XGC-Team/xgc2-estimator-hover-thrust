#pragma once

#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <string>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"
#include "hover_thrust_estimator/output/output_event_consumer.h"
#include "hover_thrust_estimator/output/ros_output_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustOutputConsumer final : public OutputEventConsumer {
   public:
    HoverThrustOutputConsumer(ros::NodeHandle& nh, RosOutputExecutor& executor,
                              HoverThrustEstimatorRuntime& runtime,
                              std::string estimate_state_topic, std::string estimate_topic,
                              std::string valid_topic, uint32_t queue_size);

    bool handle(const ::state_machine::Event& event) override;

   private:
    hover_thrust_estimator::HoverThrustEstimate makeEstimateStateMessage(
        const HoverThrustEstimatorRuntime::Output& output, const ros::Time& stamp) const;
    static std_msgs::Float64 makeEstimateMessage(const HoverThrustEstimatorRuntime::Output& output);
    static std_msgs::Bool makeValidMessage(const HoverThrustEstimatorRuntime::Output& output);

    RosOutputExecutor& executor_;
    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher estimate_state_pub_;
    ros::Publisher estimate_pub_;
    ros::Publisher valid_pub_;
};

}  // namespace hover_thrust_estimator
