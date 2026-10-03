#include <dlfcn.h>
#include <gtest/gtest.h>
#include <ros/master.h>
#include <ros/ros.h>

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "hover_thrust_estimator/input/hover_thrust_input_producer.h"
#include "hover_thrust_estimator/output/hover_thrust_output_consumer.h"
#include "hover_thrust_estimator/native/hover_thrust_wire.h"
#include "xgc_rt.h"
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>

namespace hte = hover_thrust_estimator;
namespace {

struct NativeEndpoint {
    xgc_host_api api{};
    std::array<std::deque<std::vector<uint8_t>>, 5> pending;
    std::vector<uint8_t> payload;
    std::vector<hover_thrust_native::xgc_hover_thrust_v1> publications;
    const xgc_plugin_descriptor* descriptor{nullptr};
    void* library{nullptr};
    void* instance{nullptr};
    NativeEndpoint() {
        api.abi_version = XGC_RT_ABI_VERSION;
        api.abi_minor = XGC_RT_ABI_MINOR;
        api.host = this;
        api.next = [](void* p, uint32_t port, xgc_sample_view* out) {
            auto& self = *static_cast<NativeEndpoint*>(p);
            if (self.pending[port].empty())
                return XGC_ERR_AGAIN;
            self.payload = std::move(self.pending[port].front());
            self.pending[port].pop_front();
            *out = {};
            out->data = self.payload.data();
            out->len = self.payload.size();
            return XGC_OK;
        };
        api.publish = [](void* p, uint32_t port, uint64_t, const uint8_t* data, uint32_t len) {
            if (port != 3 || len != sizeof(hover_thrust_native::xgc_hover_thrust_v1))
                return XGC_ERR;
            hover_thrust_native::xgc_hover_thrust_v1 output;
            std::memcpy(&output, data, sizeof output);
            static_cast<NativeEndpoint*>(p)->publications.push_back(output);
            return XGC_OK;
        };
        api.log = [](void*, xgc_log_level, const char*) {};
        const char* negative_control = std::getenv("HTE_NEGATIVE_CONTROL_LIBRARY");
        library = dlopen(negative_control ? negative_control : HTE_NATIVE_MODULE_PATH,
                         RTLD_NOW | RTLD_LOCAL);
        if (!library)
            throw std::runtime_error(dlerror());
        auto entry = reinterpret_cast<xgc_rt_plugin_v1_fn>(dlsym(library, "xgc_rt_plugin_v1"));
        if (!entry)
            throw std::runtime_error("missing native entry");
        descriptor = entry();
        instance = descriptor->vtbl->create(&api);
        if (!instance ||
            descriptor->vtbl->configure(instance, "time_source = \"session\"\n") != XGC_OK)
            throw std::runtime_error("native configure failed");
        if (descriptor->vtbl->activate(instance) != XGC_OK)
            throw std::runtime_error("native activate");
    }
    ~NativeEndpoint() {
        if (instance) {
            descriptor->vtbl->deactivate(instance);
            descriptor->vtbl->destroy(instance);
        }
        if (library)
            dlclose(library);
    }
    template <class T>
    void enqueue(uint32_t id, const T& sample) {
        const auto* b = reinterpret_cast<const uint8_t*>(&sample);
        pending[id].emplace_back(b, b + sizeof sample);
    }
    void tick(int64_t now_ns) {
        publications.clear();
        xgc_step_ctx ctx{};
        ctx.now = now_ns;
        for (size_t port = 0; port < pending.size(); ++port)
            if (!pending[port].empty())
                ctx.dirty_ports |= uint64_t(1) << port;
        ASSERT_EQ(descriptor->vtbl->step(instance, &ctx), XGC_OK);
        for (const auto& queue : pending)
            ASSERT_TRUE(queue.empty());
    }
};

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

TEST(HoverThrustFacades, BackwardPortFifoIsNotLaunderedByNativeBatching) {
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
    NativeEndpoint native;
    xgc_imu_v1 im{};
    im.stamp = 10;
    im.accel[2] = imu.linear_acceleration.z;
    xgc_attitude_target_v2 t{};
    t.stamp = 10;
    t.thrust = target.thrust;
    xgc_pose_v1 z{};
    z.stamp = 10;
    z.position[2] = 1;
    native.enqueue(0, im);
    native.enqueue(4, t);
    native.enqueue(2, z);
    native.tick(10000000000LL);
    ASSERT_FALSE(native.publications.empty());
    ASSERT_EQ(native.publications.back().state, hte::state_type::Airborne);
    // ROS callbacks preserve arrival order; native receives the same FIFO as
    // TWO samples in ONE port drain and ONE step. A global stamp sort fails.
    for (uint32_t ns : {200000000u, 100000000u}) {
        imu.header.stamp = ros::Time(10, ns);
        imu_pub.publish(imu);
        ASSERT_TRUE(spin_until(
            [&] { return runtime.input().imu_acc_z.stamp_sec == imu.header.stamp.toSec(); }));
        im.stamp = imu.header.stamp.toSec();
        native.enqueue(0, im);
    }
    runtime.update(10.2);
    native.tick(10200000000LL);
    ASSERT_FALSE(runtime.health().ready);
    ASSERT_NE(runtime.health().flags & hte::HoverThrustRuntimeFlag::kTimeJump, 0u);
    ASSERT_FALSE(native.publications.empty());
    const auto& n = native.publications.back();
    EXPECT_EQ(n.state, hte::state_type::SelfCheck);
    EXPECT_NE(n.flags & 8192u, 0u);
    EXPECT_EQ(n.sample_used, 0u);
    EXPECT_EQ(n.flags, runtime.snapshotOutput().flags);
    std::cout << "backward FIFO 10.2->10.1: real ROS/native flags=" << n.flags
              << " time_jump=8192 ready=false; no normalized sorting\n";
}

TEST(HoverThrustFacades, RealRosAndNativeShareEveryPublicationField) {
    ASSERT_TRUE(ros::master::check());  // No skipped test when a master is missing.
    ros::NodeHandle nh("hte_owner_equivalence");
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

    // Zero ROS headers fall back HERE, never inside runtime or native ingestion.
    ros::Time::setNow(ros::Time(2, 0));
    sensor_msgs::Imu zero;
    zero.linear_acceleration.z = 9.8066;
    imu_pub.publish(zero);
    ASSERT_TRUE(spin_until([&] { return runtime.input().imu_acc_z.received; }));
    ASSERT_DOUBLE_EQ(runtime.input().imu_acc_z.stamp_sec, 2.0);
    runtime.reset();

    NativeEndpoint native;
    size_t compared = 0;
    auto compare = [&](int64_t now_ns) {
        // Both facades receive the SAME explicit processing clock, independent
        // of their serialized source stamp and ROS wall-clock scheduling.
        const double now = static_cast<double>(now_ns) * 1e-9;
        runtime.update(now);
        native.tick(now_ns);
        const auto& pubs = runtime.publishedEstimates();
        ASSERT_EQ(pubs.size(), native.publications.size());
        for (size_t i = 0; i < pubs.size(); ++i) {
            const auto& p = pubs[i];
            const auto& n = native.publications[i];
            EXPECT_DOUBLE_EQ(n.stamp, p.stamp_sec);
            EXPECT_DOUBLE_EQ(n.hover_thrust, p.output.hover_thrust);
            EXPECT_DOUBLE_EQ(n.raw_hover_thrust, p.output.raw_hover_thrust);
            EXPECT_DOUBLE_EQ(n.initial_hover_thrust, p.output.initial_hover_thrust);
            EXPECT_DOUBLE_EQ(n.thrust_to_acceleration, p.output.thrust_to_acceleration);
            EXPECT_DOUBLE_EQ(n.last_estimate_stamp, p.output.last_estimate_stamp_sec);
            EXPECT_EQ(n.state, p.output.state);
            EXPECT_EQ(n.flags, p.output.flags);
            EXPECT_EQ(n.sample_used, p.output.sample_used ? 1u : 0u);
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
        xgc_imu_v1 im{};
        im.stamp = source_stamp;
        im.accel[2] = imu.linear_acceleration.z;
        native.enqueue(0, im);
        compare(now_ns);

        mavros_msgs::AttitudeTarget target;
        target.header.stamp = stamp;
        target.thrust = 0.4;  // Match the ROS float32 field at the native boundary.
        target.type_mask = 128 | ((i >= 150 && i < 200) ? 64 : 0);
        target.orientation.w = std::numeric_limits<double>::quiet_NaN();
        target.body_rate.x = 30;
        target_pub.publish(target);
        ASSERT_TRUE(spin_until(
            [&] { return runtime.input().normalized_thrust.stamp_sec == source_stamp; }));
        xgc_attitude_target_v2 t{};
        t.stamp = source_stamp;
        t.thrust = target.thrust;
        t.type_mask = target.type_mask;
        t.q_wxyz[0] = target.orientation.w;
        t.body_rate[0] = target.body_rate.x;
        native.enqueue(4, t);
        compare(now_ns);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.pose.position.z = i < 20 ? 0 : 1;
        pose_pub.publish(pose);
        ASSERT_TRUE(spin_until([&] { return runtime.input().altitude.stamp_sec == source_stamp; }));
        xgc_pose_v1 z{};
        z.stamp = source_stamp;
        z.position[2] = pose.pose.position.z;
        native.enqueue(2, z);
        compare(now_ns);
    }
    compare(3300000000LL);  // no new input: both genuine facades tick stale inputs
    EXPECT_EQ(runtime.currentState(), hte::state_type::SelfCheck);
    EXPECT_NE(runtime.health().flags & hte::HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_GT(compared, 150u);
    executor.stop();
    std::cout << "real ROS + native same-library publications=" << compared
              << "; normalized input/output fields equal; zero ROS stamp edge-only; stale tick "
                 "detected\n";
}
}  // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "hte_owner_facade_test", ros::init_options::AnonymousName);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
