#pragma once

#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <string>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustOutputConsumer {
   public:
    HoverThrustOutputConsumer(
        ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
        HoverThrustEstimatorRuntime& runtime, std::string estimate_state_topic,
        uint32_t queue_size);

    std::string name() const {
        return "HoverThrustOutputConsumer";
    }
    bool handle(const PublicationEvent& event);

   private:
    hover_thrust_estimator_msgs::HoverThrustEstimate makeEstimateStateMessage(
        const HoverThrustOutput& output, const ros::Time& stamp) const;

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher estimate_state_pub_;
};

}  // namespace hover_thrust_estimator
