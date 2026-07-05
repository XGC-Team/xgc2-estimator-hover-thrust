#include <gtest/gtest.h>
#include <ros/ros.h>
#include <state_machine_msgs/StateMachineTrace.h>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_node.h"
#include "hover_thrust_estimator/output/hover_thrust_debug_trace_consumer.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kGravity = estimator_limits::kDefaultGravity;

double accelerationForHover(double hover_thrust, double normalized_thrust) {
    return (kGravity / hover_thrust) * normalized_thrust;
}

HoverThrustEstimatorRuntime::Input readyInput(double sample_stamp_sec, double hover_thrust,
                                              double normalized_thrust) {
    HoverThrustEstimatorRuntime::Input input;
    input.imu_acc_z.value = accelerationForHover(hover_thrust, normalized_thrust);
    input.imu_acc_z.stamp_sec = sample_stamp_sec;
    input.imu_acc_z.received = true;
    input.imu_acc_z.finite = true;
    input.normalized_thrust.value = normalized_thrust;
    input.normalized_thrust.stamp_sec = sample_stamp_sec;
    input.normalized_thrust.received = true;
    input.normalized_thrust.finite = true;
    input.altitude.value = 1.0;
    input.altitude.stamp_sec = sample_stamp_sec;
    input.altitude.received = true;
    input.altitude.finite = true;
    input.thrust_ignored = false;
    return input;
}

void postAllInputEvents(HoverThrustEstimatorRuntime& runtime,
                        const HoverThrustEstimatorRuntime::Input& input) {
    runtime.postInputEvent(
        ::state_machine::Event(event_type::INPUT_IMU_UPDATED,
                               ::state_machine::EventTimestamp{input.normalized_thrust.stamp_sec}),
        input);
    runtime.postInputEvent(
        ::state_machine::Event(event_type::INPUT_THRUST_UPDATED,
                               ::state_machine::EventTimestamp{input.normalized_thrust.stamp_sec}),
        input);
    runtime.postInputEvent(
        ::state_machine::Event(event_type::INPUT_ALTITUDE_UPDATED,
                               ::state_machine::EventTimestamp{input.normalized_thrust.stamp_sec}),
        input);
}

TEST(HoverThrustEstimatorNodeParamsTest, LoadsAllNonDefaultPrivateParameters) {
    ros::NodeHandle nh;
    HoverThrustEstimatorNode node(nh);

    EXPECT_EQ(node.imuTopic(), "test/imu_data_non_default");
    EXPECT_EQ(node.targetAttitudeTopic(), "test/target_attitude_non_default");
    EXPECT_EQ(node.altitudeTopic(), "test/local_position_non_default");
    EXPECT_EQ(node.estimateStateTopic(), "test/hover_thrust_estimate_non_default");
    EXPECT_EQ(node.debugTraceTopic(), "test/state_machine_trace_non_default");
    EXPECT_DOUBLE_EQ(node.loopRate(), 123.0);

    const auto& config = node.runtime().config();
    EXPECT_DOUBLE_EQ(config.gravity, 9.91);
    EXPECT_DOUBLE_EQ(config.initial_hover_thrust, 0.42);
    EXPECT_DOUBLE_EQ(config.rho2, 0.987);
    EXPECT_DOUBLE_EQ(config.min_hover_thrust, 0.21);
    EXPECT_DOUBLE_EQ(config.max_hover_thrust, 0.79);
    EXPECT_DOUBLE_EQ(config.min_altitude, 1.25);
    EXPECT_DOUBLE_EQ(config.sample_timeout, 0.35);
    EXPECT_DOUBLE_EQ(config.publish_rate_hz, 45.0);
    EXPECT_DOUBLE_EQ(config.raw_update_rate_hz, 12.0);
    EXPECT_DOUBLE_EQ(config.input_rate_low_hz, 7.5);
    EXPECT_FALSE(config.filter_enabled);
    EXPECT_DOUBLE_EQ(config.filter_cutoff_hz, 4.5);

    const auto snapshot = node.runtime().snapshotOutput();
    EXPECT_DOUBLE_EQ(snapshot.initial_hover_thrust, 0.42);
    EXPECT_DOUBLE_EQ(snapshot.hover_thrust, 0.42);
}

TEST(HoverThrustDebugTraceConsumerTest, PublishesTraceThroughAsyncExecutor) {
    ros::NodeHandle nh;
    const std::string topic = ros::this_node::getName() + "/state_machine_trace";
    bool received = false;
    state_machine_msgs::StateMachineTrace received_msg;
    auto sub = nh.subscribe<state_machine_msgs::StateMachineTrace>(
        topic, 1, [&](const state_machine_msgs::StateMachineTrace::ConstPtr& msg) {
            received = true;
            received_msg = *msg;
        });
    (void)sub;

    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    ASSERT_TRUE(runtime.hasDebugTrace());

    ::state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> executor(nh);
    HoverThrustDebugTraceConsumer consumer(nh, executor, runtime, topic, 1);
    executor.start();
    ros::Duration(0.2).sleep();

    const auto events = runtime.debugOutputEvents(1.0);
    ASSERT_EQ(events.size(), 1u);
    EXPECT_TRUE(consumer.handle(events.front()));

    const ros::Time deadline = ros::Time::now() + ros::Duration(2.0);
    while (ros::ok() && !received && ros::Time::now() < deadline) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    executor.stop();

    ASSERT_TRUE(received);
    EXPECT_EQ(received_msg.machine_name, runtime.getStateMachine().name());
    EXPECT_FALSE(received_msg.events.empty());
    EXPECT_EQ(received_msg.events.front().event_id, event_type::HEALTH_ESTIMATION_READY);
}

}  // namespace
}  // namespace hover_thrust_estimator

int main(int argc, char** argv) {
    ros::init(argc, argv, "hover_thrust_estimator_node_params_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
