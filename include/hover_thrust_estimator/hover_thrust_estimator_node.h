#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <hover_thrust_estimator/HoverThrustEstimate.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <string>

#include "hover_thrust_estimator/hover_thrust_estimator.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorNode {
   public:
    explicit HoverThrustEstimatorNode(ros::NodeHandle& nh);
    void run(double frequency);
    double publishRate() const {
        return publish_rate_;
    }

   private:
    void loadParams();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void targetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void publishOutput(const HoverThrustEstimatorRuntime::Output& output, const ros::Time& stamp);
    void publishEstimate(double hover_thrust);
    void publishValid(bool valid);
    void postInputEvent(HoverThrustInputEvent event);
    void consumeOutputEvent(HoverThrustOutputEvent event, const ros::Time& stamp);
    static void updateSamplePeriod(HoverThrustEstimatorRuntime::Sample& sample, double stamp_sec);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber target_attitude_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher estimate_state_pub_;
    ros::Publisher estimate_pub_;
    ros::Publisher valid_pub_;

    HoverThrustEstimatorRuntime runtime_;
    HoverThrustEstimatorRuntime::Input runtime_input_;

    std::string imu_topic_{"mavros/imu/data"};
    std::string target_attitude_topic_{"mavros/setpoint_raw/target_attitude"};
    std::string altitude_topic_{"mavros/local_position/pose"};
    std::string estimate_state_topic_{"hover_thrust/estimate_state"};
    std::string estimate_topic_{"hover_thrust/estimate"};
    std::string valid_topic_{"hover_thrust/valid"};

    double gravity_{9.8066};
    double initial_hover_thrust_{0.5};
    double rho2_{0.998};
    double min_hover_thrust_{0.05};
    double max_hover_thrust_{0.95};
    double min_altitude_{0.5};
    double sample_timeout_{0.2};
    double publish_rate_{100.0};
    double raw_update_rate_{10.0};
    bool filter_enabled_{false};
    double filter_cutoff_hz_{2.0};
    double output_slew_rate_{0.1};
    double input_rate_low_hz_{5.0};
};

}  // namespace hover_thrust_estimator
