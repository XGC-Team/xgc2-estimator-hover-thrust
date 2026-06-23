#pragma once

#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <ros/ros.h>

#include <string>

#include "hover_thrust_estimator/common/types.h"
#include "hover_thrust_estimator/output/output_event_consumer.h"
#include "hover_thrust_estimator/output/ros_output_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class HoverThrustOutputConsumer final : public OutputEventConsumer {
   public:
    HoverThrustOutputConsumer(ros::NodeHandle& nh, RosOutputExecutor& executor,
                              HoverThrustEstimatorRuntime& runtime,
                              std::string estimate_state_topic, uint32_t queue_size);

    bool handle(const ::state_machine::Event& event) override;

   private:
    hover_thrust_estimator::HoverThrustEstimate makeEstimateStateMessage(
        const HoverThrustOutput& output, const ros::Time& stamp) const;

    RosOutputExecutor& executor_;
    HoverThrustEstimatorRuntime& runtime_;
    ros::Publisher estimate_state_pub_;
};

}  // namespace hover_thrust_estimator
