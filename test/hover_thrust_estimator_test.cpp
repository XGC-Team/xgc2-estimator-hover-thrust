#include "hover_thrust_estimator/hover_thrust_estimator.h"

#include <gtest/gtest.h>

#include <cmath>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kGravity = estimator_limits::kDefaultGravity;

double accelerationForHover(double hover_thrust, double normalized_thrust) {
    return (kGravity / hover_thrust) * normalized_thrust;
}

HoverThrustEstimator makeEstimator(bool filter_enabled, double filter_cutoff_hz) {
    HoverThrustEstimator estimator;
    HoverThrustEstimator::Config config;
    config.filter_enabled = filter_enabled;
    config.filter_cutoff_hz = filter_cutoff_hz;
    estimator.setConfig(config);
    estimator.reset(kGravity, 0.5);
    return estimator;
}

HoverThrustEstimatorRuntime::Input readyInput(double now_sec, double hover_thrust,
                                              double normalized_thrust) {
    HoverThrustEstimatorRuntime::Input input;
    input.now_sec = now_sec;
    input.imu_acc_z.value = accelerationForHover(hover_thrust, normalized_thrust);
    input.imu_acc_z.stamp_sec = now_sec;
    input.imu_acc_z.received = true;
    input.imu_acc_z.finite = true;
    input.normalized_thrust.value = normalized_thrust;
    input.normalized_thrust.stamp_sec = now_sec;
    input.normalized_thrust.received = true;
    input.normalized_thrust.finite = true;
    input.altitude.value = 1.0;
    input.altitude.stamp_sec = now_sec;
    input.altitude.received = true;
    input.altitude.finite = true;
    input.thrust_ignored = false;
    return input;
}

TEST(HoverThrustEstimatorTest, DisabledFilterKeepsEstimateEqualToRawEstimate) {
    HoverThrustEstimator estimator = makeEstimator(false, 2.0);

    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.0));
    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.02));

    EXPECT_NEAR(estimator.estimate(), estimator.rawEstimate(), 1.0e-12);
}

TEST(HoverThrustEstimatorTest, EnabledFilterSmoothsAcceptedEstimateChanges) {
    HoverThrustEstimator estimator = makeEstimator(true, 1.0);

    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.0));
    const double previous_filtered = estimator.estimate();
    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.02));

    EXPECT_GT(estimator.rawEstimate(), previous_filtered);
    EXPECT_GT(estimator.estimate(), previous_filtered);
    EXPECT_LT(estimator.estimate(), estimator.rawEstimate());
}

TEST(HoverThrustEstimatorTest, InvalidFilterCutoffFallsBackToPassThrough) {
    HoverThrustEstimator estimator = makeEstimator(true, 0.0);

    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.0));
    ASSERT_TRUE(estimator.update(accelerationForHover(0.8, 0.5), 0.5, 1.02));

    EXPECT_NEAR(estimator.estimate(), estimator.rawEstimate(), 1.0e-12);
}

TEST(HoverThrustEstimatorRuntimeTest, StartupOutputKeepsInitialHoverThrust) {
    HoverThrustEstimatorRuntime runtime;

    const auto output = runtime.output();

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kInitializing);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.5);
    EXPECT_DOUBLE_EQ(output.raw_hover_thrust, 0.5);
    EXPECT_FALSE(output.estimator_valid);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, MissingImuPublishesSafeInitialOutputWithState) {
    HoverThrustEstimatorRuntime runtime;
    HoverThrustEstimatorRuntime::Input input;
    input.now_sec = 1.0;

    const auto output = runtime.update(input);

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kWaitingForImu);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuMissing, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.5);
    EXPECT_FALSE(output.estimator_valid);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, ReadySamplesMoveToEstimatingState) {
    HoverThrustEstimatorRuntime runtime;

    const auto output = runtime.update(readyInput(1.0, 0.8, 0.5));

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kEstimating);
    EXPECT_EQ(output.flags, 0u);
    EXPECT_TRUE(output.estimator_valid);
    EXPECT_TRUE(output.sample_used);
    EXPECT_GT(output.hover_thrust, 0.5);
    EXPECT_DOUBLE_EQ(output.source_stamp_sec, 1.0);
    EXPECT_DOUBLE_EQ(output.last_estimate_stamp_sec, 1.0);
}

TEST(HoverThrustEstimatorRuntimeTest, StaleInputHoldsLastSafeEstimate) {
    HoverThrustEstimatorRuntime runtime;
    const auto estimated = runtime.update(readyInput(1.0, 0.8, 0.5));
    auto stale = readyInput(2.0, 0.7, 0.5);
    stale.imu_acc_z.stamp_sec = 1.0;
    stale.normalized_thrust.stamp_sec = 1.0;
    stale.altitude.stamp_sec = 1.0;

    const auto output = runtime.update(stale);

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kHoldingInputStale);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, estimated.hover_thrust);
    EXPECT_TRUE(output.estimator_valid);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, BelowMinimumAltitudeHoldsOutputAndReportsState) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.altitude.value = 0.1;

    const auto output = runtime.update(input);

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kBelowMinAltitude);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kBelowMinAltitude, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.5);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, IgnoredThrustReportsInvalidThrust) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.thrust_ignored = true;

    const auto output = runtime.update(input);

    EXPECT_EQ(output.state, HoverThrustRuntimeState::kInvalidThrust);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustInvalid, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.5);
    EXPECT_FALSE(output.sample_used);
}

}  // namespace
}  // namespace hover_thrust_estimator

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
