#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/types.h"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kGravity = estimator_limits::kDefaultGravity;

HoverThrustEstimatorRuntime::Input readyInput(double stamp_sec, double hover_thrust,
                                              double normalized_thrust) {
    HoverThrustEstimatorRuntime::Input input;
    input.imu_acc_z.value = (kGravity / hover_thrust) * normalized_thrust;
    input.imu_acc_z.stamp_sec = stamp_sec;
    input.imu_acc_z.received = true;
    input.imu_acc_z.finite = true;
    input.normalized_thrust.value = normalized_thrust;
    input.normalized_thrust.stamp_sec = stamp_sec;
    input.normalized_thrust.received = true;
    input.normalized_thrust.finite = true;
    input.altitude.value = 1.0;
    input.altitude.stamp_sec = stamp_sec;
    input.altitude.received = true;
    input.altitude.finite = true;
    input.thrust_ignored = false;
    return input;
}

TEST(HoverThrustRuntimeBoundary, IngestUpdatePublishAndNextWake) {
    HoverThrustEstimatorRuntime runtime;
    const auto input = readyInput(1.0, 0.8, 0.5);

    ASSERT_TRUE(runtime.ingestImu(input.imu_acc_z.value, 1.0, 1.0).ok());
    ASSERT_TRUE(runtime.ingestThrust(input.normalized_thrust.value, false, 1.0, 1.0).ok());
    ASSERT_TRUE(runtime.ingestAltitude(input.altitude.value, 1.0, 1.0).ok());
    EXPECT_TRUE(runtime.hasPendingInput());

    auto wake = runtime.nextWake(1.0);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_DOUBLE_EQ(*wake.next_deadline_sec, 1.0);

    const auto output = runtime.update(1.0);
    EXPECT_EQ(output.state, state_type::Airborne);
    EXPECT_EQ(runtime.currentState(), state_type::Airborne);
    EXPECT_TRUE(output.sample_used);
    EXPECT_EQ(output.flags, 0u);
    EXPECT_FALSE(runtime.hasPendingInput());
    EXPECT_EQ(runtime.updateIndex(), 1u);

    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);
    EXPECT_EQ(runtime.publishedEstimates().front().event.id,
              output_event_type::PUBLISH_ESTIMATE);
    EXPECT_EQ(runtime.publishedEstimates().front().event.sequence, 1u);
    EXPECT_DOUBLE_EQ(runtime.publishedEstimates().front().stamp_sec, 1.0);
    EXPECT_EQ(runtime.publishedEstimates().front().output.state, state_type::Airborne);

    ASSERT_EQ(runtime.transitionTrace().size(), 1u);
    EXPECT_EQ(runtime.transitionTrace().front().event_id,
              event_type::HEALTH_ESTIMATION_READY);
    EXPECT_EQ(runtime.transitionTrace().front().from_state, state_type::SelfCheck);
    EXPECT_EQ(runtime.transitionTrace().front().to_state, state_type::Airborne);

    wake = runtime.nextWake(1.0);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_NEAR(*wake.next_deadline_sec, 1.01, 1.0e-12);
}

TEST(HoverThrustRuntimeBoundary, StaleHealthEdgePublishesSelfCheckSameTurn) {
    HoverThrustEstimatorRuntime runtime;
    const auto input = readyInput(1.0, 0.8, 0.5);
    ASSERT_TRUE(runtime.applyInput(input, 1.0).ok());
    const auto ready = runtime.update(1.0);
    ASSERT_EQ(ready.state, state_type::Airborne);
    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);

    const auto stale = runtime.update(1.21);
    EXPECT_EQ(stale.state, state_type::SelfCheck);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);
    EXPECT_FALSE(stale.sample_used);

    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);
    EXPECT_EQ(runtime.publishedEstimates().front().output.state, state_type::SelfCheck);
    ASSERT_EQ(runtime.transitionTrace().size(), 1u);
    EXPECT_EQ(runtime.transitionTrace().front().event_id,
              event_type::HEALTH_INPUT_UNHEALTHY);
    EXPECT_EQ(runtime.transitionTrace().front().from_state, state_type::Airborne);
    EXPECT_EQ(runtime.transitionTrace().front().to_state, state_type::SelfCheck);

    // The already-passed sample timeout must not become a busy-loop wake.
    const auto wake = runtime.nextWake(1.21);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_NEAR(*wake.next_deadline_sec, 1.22, 1.0e-12);

    runtime.update(1.215);
    EXPECT_TRUE(runtime.transitionTrace().empty());
}

TEST(HoverThrustRuntimeBoundary, StaleBoundaryIsStrictAndWakeUsesTheExactEdge) {
    HoverThrustEstimatorRuntime runtime;
    ASSERT_TRUE(runtime.applyInput(readyInput(1.0, 0.8, 0.5), 1.0).ok());
    runtime.update(1.0);

    const auto still_ready = runtime.update(1.2);
    EXPECT_EQ(still_ready.state, state_type::Airborne);
    EXPECT_EQ(runtime.health().condition, HoverThrustHealthCondition::kEstimationReady);

    const double strict_edge =
        std::nextafter(1.0 + runtime.config().sample_timeout,
                       std::numeric_limits<double>::infinity());
    const auto wake = runtime.nextWake(1.2);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_DOUBLE_EQ(*wake.next_deadline_sec, strict_edge);

    const auto stale = runtime.update(strict_edge);
    EXPECT_EQ(stale.state, state_type::SelfCheck);
    EXPECT_EQ(runtime.health().condition, HoverThrustHealthCondition::kInputUnhealthy);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(stale.flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);
}

TEST(HoverThrustRuntimeBoundary, StartupWakeAndApplyInputBatch) {
    HoverThrustEstimatorRuntime runtime;
    auto wake = runtime.nextWake(0.0);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_DOUBLE_EQ(*wake.next_deadline_sec, 0.0);

    const auto input = readyInput(0.001, 0.8, 0.5);
    ASSERT_TRUE(runtime.applyInput(input, 0.001).ok());
    EXPECT_TRUE(runtime.hasPendingInput());
    const auto output = runtime.update(0.001);
    EXPECT_EQ(output.state, state_type::Airborne);
    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);
    ASSERT_EQ(runtime.transitionTrace().size(), 1u);

    wake = runtime.nextWake(0.001);
    ASSERT_TRUE(wake.next_deadline_sec.has_value());
    EXPECT_NEAR(*wake.next_deadline_sec, 0.011, 1.0e-12);
}

TEST(HoverThrustRuntimeBoundary, PublicationSequenceIsMonotonicAcrossTurns) {
    HoverThrustEstimatorRuntime runtime;
    ASSERT_TRUE(runtime.applyInput(readyInput(1.0, 0.8, 0.5), 1.0).ok());
    runtime.update(1.0);
    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);
    const uint64_t first_sequence = runtime.publishedEstimates().front().event.sequence;

    runtime.update(1.011);
    ASSERT_EQ(runtime.publishedEstimates().size(), 1u);
    EXPECT_EQ(runtime.publishedEstimates().front().event.sequence, first_sequence + 1);
    EXPECT_DOUBLE_EQ(runtime.publishedEstimates().front().stamp_sec, 1.011);
}

}  // namespace
}  // namespace hover_thrust_estimator
