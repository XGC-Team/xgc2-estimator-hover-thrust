#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "hover_thrust_estimator/hover_thrust_estimator.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorNode {
public:
    explicit HoverThrustEstimatorNode(ros::NodeHandle& nh);

private:
    struct TopicSample {
        ros::Time stamp;
        bool received{false};
    };

    void loadParams();
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void targetAttitudeCallback(const mavros_msgs::AttitudeTarget::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void updateCallback(const ros::TimerEvent& event);

    bool sampleFresh(const TopicSample& sample, const ros::Time& now) const;
    bool sampleReady(const ros::Time& now) const;
    void publishEstimate(const ros::Time& stamp);
    void publishValid(bool valid);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber imu_sub_;
    ros::Subscriber target_attitude_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher estimate_pub_;
    ros::Publisher valid_pub_;
    ros::Timer update_timer_;

    HoverThrustEstimator estimator_;

    std::string imu_topic_{"mavros/imu/data"};
    std::string target_attitude_topic_{"mavros/setpoint_raw/target_attitude"};
    std::string altitude_topic_{"mavros/local_position/pose"};
    std::string estimate_topic_{"hover_thrust/estimate"};
    std::string valid_topic_{"hover_thrust/valid"};

    double gravity_{9.8066};
    double initial_hover_thrust_{0.5};
    double rho2_{0.998};
    double min_hover_thrust_{0.05};
    double max_hover_thrust_{0.95};
    double min_altitude_{0.5};
    double sample_timeout_{0.2};
    double publish_rate_{50.0};

    double latest_acc_z_{0.0};
    double latest_thrust_{0.0};
    double latest_altitude_{0.0};

    TopicSample imu_sample_;
    TopicSample thrust_sample_;
    TopicSample altitude_sample_;
    bool thrust_valid_{false};
    bool last_published_valid_{false};
};

}  // namespace hover_thrust_estimator
