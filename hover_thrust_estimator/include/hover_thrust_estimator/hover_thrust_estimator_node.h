#pragma once

#include <ros/ros.h>

#include <memory>
#include <state_machine/runtime/async_task_executor.hpp>
#include <state_machine/runtime/event_dispatcher.hpp>
#include <string>
#include <vector>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"
#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"
#include "hover_thrust_estimator/output/hover_thrust_debug_trace_consumer.h"
#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorNode {
   public:
    explicit HoverThrustEstimatorNode(ros::NodeHandle& nh);
    ~HoverThrustEstimatorNode();
    void run(double frequency);
    double loopRate() const {
        return loop_rate_;
    }
    const std::string& imuTopic() const {
        return imu_topic_;
    }
    const std::string& targetAttitudeTopic() const {
        return target_attitude_topic_;
    }
    const std::string& altitudeTopic() const {
        return altitude_topic_;
    }
    const std::string& estimateStateTopic() const {
        return estimate_state_topic_;
    }
    const std::string& debugTraceTopic() const {
        return debug_trace_topic_;
    }
    const HoverThrustEstimatorRuntime& runtime() const {
        return runtime_;
    }

   private:
    void loadParams();
    void handlePublicationEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    HoverThrustEstimatorRuntime runtime_;
    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> output_event_executor_;
    ::state_machine::runtime::EventDispatcher output_event_dispatcher_;
    std::unique_ptr<HoverThrustInputProducer> input_producer_;

    std::string imu_topic_{"mavros/imu/data"};
    std::string target_attitude_topic_{"mavros/setpoint_raw/target_attitude"};
    std::string altitude_topic_{"mavros/local_position/pose"};
    std::string estimate_state_topic_{"hover_thrust/estimate_state"};
    std::string debug_trace_topic_{"hover_thrust/debug/state_machine_trace"};

    double loop_rate_{1000.0};
    HoverThrustEstimatorConfig estimator_config_{};
};

}  // namespace hover_thrust_estimator
