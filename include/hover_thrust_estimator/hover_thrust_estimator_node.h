#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"
#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"
#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"
#include "hover_thrust_estimator/output/output_event_consumer.h"
#include "hover_thrust_estimator/output/ros_output_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorNode {
   public:
    explicit HoverThrustEstimatorNode(ros::NodeHandle& nh);
    ~HoverThrustEstimatorNode();
    void run(double frequency);
    double loopRate() const {
        return loop_rate_;
    }

   private:
    void loadParams();
    void dispatchOutputEvents(const std::vector<::state_machine::Event>& events);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    HoverThrustEstimatorRuntime runtime_;
    RosOutputExecutor output_event_executor_;
    OutputEventDispatcher output_event_dispatcher_;
    std::unique_ptr<HoverThrustInputProducer> input_producer_;

    std::string imu_topic_{"mavros/imu/data"};
    std::string target_attitude_topic_{"mavros/setpoint_raw/target_attitude"};
    std::string altitude_topic_{"mavros/local_position/pose"};
    std::string estimate_state_topic_{"hover_thrust/estimate_state"};

    double loop_rate_{1000.0};
    HoverThrustEstimatorConfig estimator_config_{};
};

}  // namespace hover_thrust_estimator
