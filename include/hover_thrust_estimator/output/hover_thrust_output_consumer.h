#pragma once

#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>

#include "hover_thrust_estimator/common/types.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class HoverThrustOutputConsumer final : public ::state_machine::runtime::EventConsumer {
   public:
    HoverThrustOutputConsumer(
        ros::NodeHandle& nh, ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor,
        HoverThrustEstimatorRuntime& runtime, std::string estimate_state_topic,
        uint32_t queue_size);

    std::string name() const override {
        return "HoverThrustOutputConsumer";
    }
    bool handle(const ::state_machine::Event& event) override;

   private:
    hover_thrust_estimator::HoverThrustEstimate makeEstimateStateMessage(
        const HoverThrustOutput& output, const ros::Time& stamp) const;

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>& executor_;
    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher estimate_state_pub_;
};

}  // namespace hover_thrust_estimator
