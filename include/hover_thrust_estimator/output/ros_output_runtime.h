#pragma once

#include <ros/ros.h>

#include <state_machine/runtime/async_task_executor.hpp>

namespace hover_thrust_estimator {

using RosOutputTask = ::state_machine::runtime::Task<ros::NodeHandle>;
using RosOutputExecutor = ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle>;
using RosLambdaOutputTask = ::state_machine::runtime::LambdaTask<ros::NodeHandle>;

}  // namespace hover_thrust_estimator
