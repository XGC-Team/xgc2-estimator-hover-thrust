#pragma once

#include <ros/ros.h>
#include <state_machine_msgs/StateMachineTrace.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class HoverThrustDebugTraceConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    HoverThrustDebugTraceConsumer(
        ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
        HoverThrustEstimatorRuntime& runtime, std::string debug_trace_topic, uint32_t queue_size);

    std::string name() const override {
        return "HoverThrustDebugTraceConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    state_machine_msgs::StateMachineTrace makeTraceMessage(const ros::Time& stamp) const;

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher debug_trace_pub_;
};

}  // namespace hover_thrust_estimator
