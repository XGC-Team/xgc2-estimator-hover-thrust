// The ROS facade (HoverThrustInputProducer and HoverThrustOutputConsumer) on real topics of a
// private ROS master, against the runtime it drives: what the producer ingests is what was
// published, in arrival order, and what the consumer publishes carries the runtime's output.
#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"
#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"

namespace hte = hover_thrust_estimator;
namespace {

template <class Condition>
bool spin_until(Condition ready) {
    const auto until = ros::WallTime::now() + ros::WallDuration(3);
    do {
        ros::spinOnce();
        if (ready())
            return true;
        ros::WallDuration(0.001).sleep();
    } while (ros::WallTime::now() < until);
    return false;
}

TEST(HoverThrustRosFacade, BackwardStampsAreIngestedInArrivalOrder) {
    ASSERT_TRUE(ros::master::check());
    ros::NodeHandle nh("hte_backward_fifo");
    hte::HoverThrustEstimatorRuntime runtime;
    hte::HoverThrustInputProducer input(nh, "imu", "target", "pose", 10, runtime);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    auto target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("target", 10);
    auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 10);
    ASSERT_TRUE(spin_until([&] {
        return imu_pub.getNumSubscribers() && target_pub.getNumSubscribers() &&
               pose_pub.getNumSubscribers();
    }));
    sensor_msgs::Imu imu;
    imu.header.stamp = ros::Time(10, 0);
    imu.linear_acceleration.z = 9.8066;
    mavros_msgs::AttitudeTarget target;
    target.header.stamp = imu.header.stamp;
    target.thrust = .3;
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = imu.header.stamp;
    pose.pose.position.z = 1;
    imu_pub.publish(imu);
    target_pub.publish(target);
    pose_pub.publish(pose);
    ASSERT_TRUE(spin_until([&] {
        return runtime.input().imu_acc_z.received && runtime.input().normalized_thrust.received &&
               runtime.input().altitude.received;
    }));
    runtime.update(10);
    ASSERT_TRUE(runtime.health().ready);
    EXPECT_EQ(runtime.currentState(), hte::state_type::Airborne);
    // ROS callbacks preserve arrival order: a sample older than its predecessor is a clock fault
    // of the source, not something to sort away.
    for (uint32_t ns : {200000000u, 100000000u}) {
        imu.header.stamp = ros::Time(10, ns);
        imu_pub.publish(imu);
        ASSERT_TRUE(spin_until(
            [&] { return runtime.input().imu_acc_z.stamp_sec == imu.header.stamp.toSec(); }));
    }
    runtime.update(10.2);
    ASSERT_FALSE(runtime.health().ready);
    ASSERT_NE(runtime.health().flags & hte::HoverThrustRuntimeFlag::kTimeJump, 0u);
    const hte::HoverThrustOutput output = runtime.snapshotOutput();
    EXPECT_EQ(runtime.currentState(), hte::state_type::SelfCheck);
    EXPECT_NE(output.flags & hte::HoverThrustRuntimeFlag::kTimeJump, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustRosFacade, PublishedEstimatesCarryTheRuntimeOutput) {
    ASSERT_TRUE(ros::master::check());  // No skipped test when a master is missing.
    ros::NodeHandle nh("hte_ros_facade");
    hte::HoverThrustEstimatorRuntime runtime;
    hte::HoverThrustInputProducer input(nh, "imu", "target", "pose", 100, runtime);
    auto imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
    auto target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("target", 100);
    auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    std::vector<hover_thrust_estimator_msgs::HoverThrustEstimate> ros_outputs;
    auto sub = nh.subscribe<hover_thrust_estimator_msgs::HoverThrustEstimate>(
        "estimate", 100, [&](const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& m) {
            ros_outputs.push_back(*m);
        });
    state_machine::runtime::AsyncTaskExecutor<ros::NodeHandle> executor(nh);
    hte::HoverThrustOutputConsumer output(nh, executor, runtime, "estimate", 100);
    executor.start();
    ASSERT_TRUE(spin_until([&] {
        return imu_pub.getNumSubscribers() && target_pub.getNumSubscribers() &&
               pose_pub.getNumSubscribers() && sub.getNumPublishers();
    }));

    // Zero ROS headers fall back HERE, never inside the runtime's ingestion.
    ros::Time::setNow(ros::Time(2, 0));
    sensor_msgs::Imu zero;
    zero.linear_acceleration.z = 9.8066;
    imu_pub.publish(zero);
    ASSERT_TRUE(spin_until([&] { return runtime.input().imu_acc_z.received; }));
    ASSERT_DOUBLE_EQ(runtime.input().imu_acc_z.stamp_sec, 2.0);
    runtime.reset();

    size_t compared = 0;
    auto compare = [&](int64_t now_ns) {
        // The runtime is updated at an explicit processing clock, independent of the serialized
        // source stamp and of ROS wall-clock scheduling.
        const double now = static_cast<double>(now_ns) * 1e-9;
        runtime.update(now);
        for (const auto& p : runtime.publishedEstimates()) {
            const size_t previous = ros_outputs.size();
            ASSERT_TRUE(output.handle(p.event));
            ASSERT_TRUE(spin_until([&] { return ros_outputs.size() == previous + 1; }));
            const auto& r = ros_outputs.back();
            EXPECT_EQ(r.header.stamp.toNSec(), uint64_t(now_ns));
            EXPECT_EQ(r.state, p.output.state - hte::state_type::SelfCheck);
            EXPECT_EQ(r.flags, p.output.flags);
            EXPECT_DOUBLE_EQ(r.hover_thrust, p.output.hover_thrust);
            ++compared;
        }
    };
    for (int i = 1; i <= 400; ++i) {
        const int64_t now_ns = 1000000000LL + i * 5000000LL;
        ros::Time stamp;
        stamp.fromNSec(now_ns);
        const double source_stamp = stamp.toSec();
        sensor_msgs::Imu imu;
        imu.header.stamp = stamp;
        imu.linear_acceleration.z = 9.8066 / 0.42 * 0.4;
        imu_pub.publish(imu);
        ASSERT_TRUE(
            spin_until([&] { return runtime.input().imu_acc_z.stamp_sec == source_stamp; }));
        compare(now_ns);

        mavros_msgs::AttitudeTarget target;
        target.header.stamp = stamp;
        target.thrust = 0.4;
        target.type_mask = 128 | ((i >= 150 && i < 200) ? 64 : 0);
        target.orientation.w = std::numeric_limits<double>::quiet_NaN();
        target.body_rate.x = 30;
        target_pub.publish(target);
        ASSERT_TRUE(spin_until(
            [&] { return runtime.input().normalized_thrust.stamp_sec == source_stamp; }));
        compare(now_ns);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.pose.position.z = i < 20 ? 0 : 1;
        pose_pub.publish(pose);
        ASSERT_TRUE(spin_until([&] { return runtime.input().altitude.stamp_sec == source_stamp; }));
        compare(now_ns);
    }
    compare(3300000000LL);  // no new input: the stale inputs are detected
    EXPECT_EQ(runtime.currentState(), hte::state_type::SelfCheck);
    EXPECT_NE(runtime.health().flags & hte::HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_GT(compared, 150u);
    executor.stop();
    std::cout << "ROS publications=" << compared
              << "; state, flags, hover thrust and stamp equal the runtime's output; zero ROS "
                 "stamp handled at the edge only; stale tick detected\n";
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "hte_ros_facade_test", ros::init_options::AnonymousName);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
