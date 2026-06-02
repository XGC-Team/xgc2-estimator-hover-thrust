#include <ros/ros.h>

#include "hover_thrust_estimator/hover_thrust_estimator_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_thrust_estimator_node");
    ros::NodeHandle nh;

    hover_thrust_estimator::HoverThrustEstimatorNode node(nh);
    ros::spin();
    return 0;
}
