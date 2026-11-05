#pragma once

#include <ros/ros.h>
#include <state_machine_msgs/StateMachineTrace.h>

#include <string>
#include <vector>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustDebugTraceConsumer {
   public:
    HoverThrustDebugTraceConsumer(ros::NodeHandle& nh, HoverThrustEstimatorRuntime& runtime,
                                  std::string debug_trace_topic, uint32_t queue_size);

    std::string name() const {
        return "HoverThrustDebugTraceConsumer";
    }
    bool handle(const std::vector<TransitionTrace>& traces);

   private:
    state_machine_msgs::StateMachineTrace makeTraceMessage(
        const ros::Time& stamp, const std::vector<TransitionTrace>& traces) const;

    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher debug_trace_pub_;
};

}  // namespace hover_thrust_estimator
