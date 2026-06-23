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
    std::string estimate_topic_{"hover_thrust/estimate"};

    double gravity_{9.8066};
    double initial_hover_thrust_{0.3};
    double rho2_{0.998};
    double min_hover_thrust_{0.15};
    double max_hover_thrust_{0.85};
    double min_altitude_{0.5};
    double sample_timeout_{0.2};
    double loop_rate_{1000.0};
    double publish_rate_{100.0};
    double raw_update_rate_{10.0};
    bool filter_enabled_{true};
    double filter_cutoff_hz_{2.0};
    double input_rate_low_hz_{5.0};
};

}  // namespace hover_thrust_estimator
