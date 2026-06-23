#include <ros/ros.h>

#include <exception>

#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_thrust_estimator_node");
    ros::NodeHandle nh;

    ROS_INFO("========================================");
    ROS_INFO("  Hover Thrust Estimator Node Starting");
    ROS_INFO("========================================");

    try {
        hover_thrust_estimator::HoverThrustEstimatorNode node(nh);

        ROS_INFO("Hover Thrust Estimator Node initialized successfully");
        ROS_INFO("Publishing hover thrust state on topic: hover_thrust/estimate_state");
        ROS_INFO("========================================");

        node.run(node.publishRate());
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in main: %s", e.what());
        return 1;
    }

    return 0;
}
