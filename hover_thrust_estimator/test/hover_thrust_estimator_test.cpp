#include "hover_thrust_estimator/hover_thrust_estimator.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "hover_thrust_estimator/common/config_utils.h"
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
    estimator.reset(kGravity, 0.3);
    return estimator;
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
                        const HoverThrustEstimatorRuntime::Input& input, double event_time_sec) {
    runtime.postInputEvent(::state_machine::Event(event_type::INPUT_IMU_UPDATED,
                                                  ::state_machine::EventTimestamp{event_time_sec}),
                           input);
    runtime.postInputEvent(::state_machine::Event(event_type::INPUT_THRUST_UPDATED,
                                                  ::state_machine::EventTimestamp{event_time_sec}),
                           input);
    runtime.postInputEvent(::state_machine::Event(event_type::INPUT_ALTITUDE_UPDATED,
                                                  ::state_machine::EventTimestamp{event_time_sec}),
                           input);
}

void postAllInputEvents(HoverThrustEstimatorRuntime& runtime,
                        const HoverThrustEstimatorRuntime::Input& input) {
    postAllInputEvents(runtime, input, input.normalized_thrust.stamp_sec);
}

bool committedTransitionForEvent(const std::vector<::state_machine::ProcessedEventRecord>& records,
                                 ::state_machine::EventId event_id, ::state_machine::StateId from,
                                 ::state_machine::StateId to) {
    for (const auto& record : records) {
        if (record.triggered_transition && record.event.id == event_id &&
            record.from_state == from && record.to_state == to) {
            return true;
        }
    }
    return false;
}

bool committedAnyHealthTransition(
    const std::vector<::state_machine::ProcessedEventRecord>& records) {
    for (const auto& record : records) {
        if (!record.triggered_transition) {
            continue;
        }
        if (record.event.id == event_type::HEALTH_INPUT_UNHEALTHY ||
            record.event.id == event_type::HEALTH_BELOW_MIN_ALTITUDE ||
            record.event.id == event_type::HEALTH_ESTIMATION_READY) {
            return true;
        }
    }
    return false;
}

bool debugTraceHasEvent(const HoverThrustEstimatorRuntime& runtime,
                        ::state_machine::EventTraceRecord::Kind kind,
                        ::state_machine::EventId event_id) {
    for (const auto& record : runtime.debugTraceRecords()) {
        if (record.trace.kind == kind && record.trace.event.id == event_id) {
            return true;
        }
    }
    return false;
}

bool debugTraceHasTransition(const HoverThrustEstimatorRuntime& runtime,
                             ::state_machine::EventId event_id, ::state_machine::StateId from,
                             ::state_machine::StateId to) {
    for (const auto& record : runtime.debugTraceRecords()) {
        if (record.trace.kind == ::state_machine::EventTraceRecord::Kind::kTransitionCommitted &&
            record.trace.event.id == event_id && record.trace.from_state == from &&
            record.trace.to_state == to) {
            return true;
        }
    }
    return false;
}

HoverThrustEstimatorRuntime::Output advanceFilterForPublish(HoverThrustEstimatorRuntime& runtime,
                                                            double now_sec) {
    runtime.outputModel().driveTowardTarget(now_sec);
    return runtime.refreshOutputSnapshot();
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

TEST(HoverThrustEstimatorTest, RawEstimateIsClampedToConfiguredBounds) {
    HoverThrustEstimator estimator = makeEstimator(false, 0.0);

    ASSERT_TRUE(estimator.update(accelerationForHover(2.0, 0.5), 0.5, 1.0));
    EXPECT_NEAR(estimator.rawEstimate(), 0.85, 1.0e-12);

    ASSERT_TRUE(estimator.update(accelerationForHover(0.01, 0.5), 0.5, 1.02));
    EXPECT_NEAR(estimator.rawEstimate(), 0.15, 1.0e-12);
}

TEST(HoverThrustConfigTest, NormalizeLoopAndEstimatorRatesInPlace) {
    HoverThrustEstimatorConfig config;
    config.publish_rate_hz = 6000.0;
    config.raw_update_rate_hz = 6000.0;
    double loop_rate_hz = 10000.0;

    config_utils::normalizeLoopAndEstimatorRates(loop_rate_hz, config);

    EXPECT_DOUBLE_EQ(loop_rate_hz, config_utils::kMaxLoopRateHz);
    EXPECT_DOUBLE_EQ(config.publish_rate_hz, config_utils::kMaxLoopRateHz);
    EXPECT_DOUBLE_EQ(config.raw_update_rate_hz, config_utils::kMaxLoopRateHz);

    config.publish_rate_hz = -1.0;
    config.raw_update_rate_hz = 0.0;
    loop_rate_hz = std::numeric_limits<double>::quiet_NaN();

    config_utils::normalizeLoopAndEstimatorRates(loop_rate_hz, config);

    EXPECT_DOUBLE_EQ(loop_rate_hz, config_utils::kDefaultLoopRateHz);
    EXPECT_DOUBLE_EQ(config.publish_rate_hz, config_utils::kDefaultPublishRateHz);
    EXPECT_DOUBLE_EQ(config.raw_update_rate_hz, config_utils::kDefaultRawUpdateRateHz);
}

TEST(HoverThrustEstimatorRuntimeTest, StartupOutputKeepsInitialHoverThrust) {
    HoverThrustEstimatorRuntime runtime;

    const auto output = runtime.output(0.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.3);
    EXPECT_EQ(output.flags, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, PublishOutputIsDrivenByStateGate) {
    HoverThrustEstimatorRuntime runtime;

    runtime.update(1.0);
    ASSERT_EQ(runtime.getStateMachine().currentOutputEvents().size(), 1u);
    EXPECT_EQ(runtime.getStateMachine().currentOutputEvents().front().id,
              output_event_type::PUBLISH_ESTIMATE);

    runtime.update(1.005);
    EXPECT_TRUE(runtime.getStateMachine().currentOutputEvents().empty());

    runtime.update(1.011);
    ASSERT_EQ(runtime.getStateMachine().currentOutputEvents().size(), 1u);
    EXPECT_EQ(runtime.getStateMachine().currentOutputEvents().front().id,
              output_event_type::PUBLISH_ESTIMATE);
}

TEST(HoverThrustEstimatorRuntimeTest, MissingImuPublishesSelfCheckOutputWithFlags) {
    HoverThrustEstimatorRuntime runtime;
    HoverThrustEstimatorRuntime::Input input;
    constexpr double event_time_sec = 1.0;

    runtime.postInputEvent(::state_machine::Event(event_type::INPUT_THRUST_UPDATED,
                                                  ::state_machine::EventTimestamp{event_time_sec}),
                           input);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuMissing, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.3);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, ReadySamplesMoveToAirborneState) {
    HoverThrustEstimatorRuntime runtime;

    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::Airborne);
    EXPECT_EQ(output.flags, 0u);
    EXPECT_TRUE(output.sample_used);
    EXPECT_GT(output.raw_hover_thrust, 0.3);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.3);
    EXPECT_DOUBLE_EQ(output.source_stamp_sec, 1.0);
    EXPECT_DOUBLE_EQ(output.last_estimate_stamp_sec, 1.0);
    EXPECT_TRUE(runtime.hasDebugTrace());
    EXPECT_TRUE(debugTraceHasEvent(runtime,
                                   ::state_machine::EventTraceRecord::Kind::kInternalEventGenerated,
                                   event_type::HEALTH_ESTIMATION_READY));
    EXPECT_TRUE(debugTraceHasTransition(runtime, event_type::HEALTH_ESTIMATION_READY,
                                        state_type::SelfCheck, state_type::Airborne));
    ASSERT_EQ(runtime.debugOutputEvents(1.0).size(), 1u);
    EXPECT_EQ(runtime.debugOutputEvents(1.0).front().id, output_event_type::PUBLISH_DEBUG_TRACE);

    const auto filtered_output = advanceFilterForPublish(runtime, 1.0);
    EXPECT_GT(filtered_output.hover_thrust, 0.3);
}

TEST(HoverThrustEstimatorRuntimeTest, StaleInputHoldsLastSafeEstimate) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    auto stale = readyInput(2.0, 0.7, 0.5);
    stale.imu_acc_z.stamp_sec = 1.0;
    stale.normalized_thrust.stamp_sec = 1.0;
    stale.altitude.stamp_sec = 1.0;

    postAllInputEvents(runtime, stale, 2.0);
    const auto output = runtime.update(2.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, TickDetectsStaleInputsWithoutNewInputEvents) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    ASSERT_EQ(runtime.output(1.0).state, state_type::Airborne);

    runtime.update(1.21);

    EXPECT_EQ(runtime.currentState(), state_type::SelfCheck);
    EXPECT_EQ(runtime.health().condition, HoverThrustHealthCondition::kInputUnhealthy);
    EXPECT_NE(runtime.health().flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(runtime.health().flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(runtime.health().flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);

    const auto output = runtime.update(1.211);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustStale, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kAltitudeStale, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, HealthEventsAreTriggeredByConditionEdges) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    ASSERT_EQ(runtime.currentState(), state_type::Airborne);
    ASSERT_EQ(runtime.health().condition, HoverThrustHealthCondition::kEstimationReady);

    const size_t before_same_condition_update = runtime.getStateMachine().generatedEventCount();
    runtime.update(1.005);
    EXPECT_EQ(runtime.getStateMachine().generatedEventCount(), before_same_condition_update);
    EXPECT_TRUE(runtime.getStateMachine().currentEvents().empty());

    const size_t before_condition_edge = runtime.getStateMachine().generatedEventCount();
    runtime.update(1.21);
    EXPECT_EQ(runtime.getStateMachine().generatedEventCount(), before_condition_edge + 1);
    EXPECT_EQ(runtime.currentState(), state_type::SelfCheck);
    EXPECT_EQ(runtime.health().condition, HoverThrustHealthCondition::kInputUnhealthy);
    EXPECT_TRUE(committedTransitionForEvent(runtime.getStateMachine().currentEvents(),
                                            event_type::HEALTH_INPUT_UNHEALTHY,
                                            state_type::Airborne, state_type::SelfCheck));
    EXPECT_TRUE(debugTraceHasEvent(runtime,
                                   ::state_machine::EventTraceRecord::Kind::kInternalEventGenerated,
                                   event_type::HEALTH_INPUT_UNHEALTHY));
    EXPECT_TRUE(debugTraceHasTransition(runtime, event_type::HEALTH_INPUT_UNHEALTHY,
                                        state_type::Airborne, state_type::SelfCheck));

    runtime.update(1.215);
    EXPECT_FALSE(committedTransitionForEvent(runtime.getStateMachine().currentEvents(),
                                             event_type::HEALTH_INPUT_UNHEALTHY,
                                             state_type::Airborne, state_type::SelfCheck));
    EXPECT_FALSE(runtime.hasDebugTrace());
}

TEST(HoverThrustEstimatorRuntimeTest, StableReadyInputDoesNotEmitHealthTransitionsAtOneKilohertz) {
    HoverThrustEstimatorRuntime runtime;
    HoverThrustEstimatorRuntime::Config config;
    config.publish_rate_hz = 0.5;
    runtime.setConfig(config);

    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    ASSERT_EQ(runtime.currentState(), state_type::Airborne);
    ASSERT_EQ(runtime.health().condition, HoverThrustHealthCondition::kEstimationReady);

    for (int i = 1; i <= 1000; ++i) {
        const double stamp_sec = 1.0 + 0.001 * static_cast<double>(i);
        auto input = readyInput(stamp_sec, 0.8, 0.5);
        input.imu_acc_z.period_sec = 0.001;
        input.normalized_thrust.period_sec = 0.001;
        input.altitude.period_sec = 0.001;

        const size_t before_input_events = runtime.getStateMachine().generatedEventCount();
        postAllInputEvents(runtime, input);
        ASSERT_EQ(runtime.getStateMachine().generatedEventCount(), before_input_events + 3)
            << "iteration " << i;
        runtime.update(stamp_sec);

        EXPECT_EQ(runtime.getStateMachine().generatedEventCount(), before_input_events + 3)
            << "iteration " << i;
        EXPECT_EQ(runtime.currentState(), state_type::Airborne) << "iteration " << i;
        EXPECT_EQ(runtime.health().condition, HoverThrustHealthCondition::kEstimationReady)
            << "iteration " << i;
        EXPECT_FALSE(committedAnyHealthTransition(runtime.getStateMachine().currentEvents()))
            << "iteration " << i;
        EXPECT_FALSE(runtime.hasDebugTrace()) << "iteration " << i;
        EXPECT_TRUE(runtime.debugOutputEvents(stamp_sec).empty()) << "iteration " << i;
    }
}

TEST(HoverThrustEstimatorRuntimeTest, TickStaleSelfCheckKeepsPublishingOutputEvents) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    advanceFilterForPublish(runtime, 1.0);

    runtime.update(1.21);
    EXPECT_EQ(runtime.output(1.21).state, state_type::SelfCheck);

    runtime.update(1.221);

    ASSERT_EQ(runtime.getStateMachine().currentOutputEvents().size(), 1u);
    EXPECT_EQ(runtime.getStateMachine().currentOutputEvents().front().id,
              output_event_type::PUBLISH_ESTIMATE);
    const auto filtered_output = advanceFilterForPublish(runtime, 1.221);
    EXPECT_EQ(filtered_output.state, state_type::SelfCheck);
    EXPECT_GT(filtered_output.hover_thrust, 0.3);
}

TEST(HoverThrustEstimatorRuntimeTest, SelfCheckReturnsToDefaultWithoutOutputJump) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    runtime.update(1.0);
    const auto airborne_output = advanceFilterForPublish(runtime, 1.0);

    auto stale = readyInput(1.21, 0.7, 0.5);
    stale.imu_acc_z.stamp_sec = 1.0;
    stale.normalized_thrust.stamp_sec = 1.0;
    stale.altitude.stamp_sec = 1.0;

    postAllInputEvents(runtime, stale, 1.21);
    runtime.update(1.21);
    const auto output = advanceFilterForPublish(runtime, 1.21);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_LT(output.hover_thrust, airborne_output.hover_thrust);
    EXPECT_GT(output.hover_thrust, 0.3);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, BelowMinimumAltitudeHoldsOutputAndReportsState) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.altitude.value = 0.1;

    postAllInputEvents(runtime, input);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::Ground);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kBelowMinAltitude, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.3);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, GroundKeepsFilteringTowardFrozenTarget) {
    HoverThrustEstimatorRuntime runtime;
    postAllInputEvents(runtime, readyInput(0.001, 0.8, 0.5));
    runtime.update(0.001);
    const auto airborne_output = advanceFilterForPublish(runtime, 0.001);

    auto ground_input = readyInput(0.02, 0.7, 0.5);
    ground_input.altitude.value = 0.1;
    postAllInputEvents(runtime, ground_input);
    runtime.update(0.02);
    const auto ground_output = advanceFilterForPublish(runtime, 0.02);

    EXPECT_EQ(ground_output.state, state_type::Ground);
    EXPECT_GT(ground_output.hover_thrust, airborne_output.hover_thrust);
    EXPECT_GT(ground_output.raw_hover_thrust, 0.3);
}

TEST(HoverThrustEstimatorRuntimeTest, IgnoredThrustReportsInvalidThrust) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.thrust_ignored = true;

    postAllInputEvents(runtime, input);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustInvalid, 0u);
    EXPECT_DOUBLE_EQ(output.hover_thrust, 0.3);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, TimeJumpReturnsToSelfCheckWithFlag) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.imu_acc_z.stamp_sec = 1.2;
    input.normalized_thrust.stamp_sec = 1.2;
    input.altitude.stamp_sec = 1.2;

    postAllInputEvents(runtime, input, 1.0);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kTimeJump, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, LowInputRateReturnsToSelfCheckWithFlag) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.imu_acc_z.period_sec = 1.0;
    input.normalized_thrust.period_sec = 1.0;
    input.altitude.period_sec = 1.0;

    postAllInputEvents(runtime, input);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kInputRateLow, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, NonFiniteInputsReturnToSelfCheckWithFlags) {
    HoverThrustEstimatorRuntime runtime;
    auto input = readyInput(1.0, 0.8, 0.5);
    input.imu_acc_z.value = std::numeric_limits<double>::quiet_NaN();
    input.imu_acc_z.finite = false;
    input.normalized_thrust.value = std::numeric_limits<double>::quiet_NaN();
    input.normalized_thrust.finite = false;
    input.altitude.value = std::numeric_limits<double>::quiet_NaN();
    input.altitude.finite = false;

    postAllInputEvents(runtime, input);
    const auto output = runtime.update(1.0);

    EXPECT_EQ(output.state, state_type::SelfCheck);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kImuInvalid, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kThrustInvalid, 0u);
    EXPECT_NE(output.flags & HoverThrustRuntimeFlag::kAltitudeInvalid, 0u);
    EXPECT_FALSE(output.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, GroundAndAirborneCanRecoverBothDirections) {
    HoverThrustEstimatorRuntime runtime;
    auto ground_input = readyInput(1.0, 0.8, 0.5);
    ground_input.altitude.value = 0.1;

    postAllInputEvents(runtime, ground_input);
    const auto ground_output = runtime.update(1.0);
    ASSERT_EQ(ground_output.state, state_type::Ground);

    postAllInputEvents(runtime, readyInput(1.02, 0.8, 0.5));
    const auto airborne_output = runtime.update(1.02);
    EXPECT_EQ(airborne_output.state, state_type::Airborne);
    EXPECT_TRUE(airborne_output.sample_used);

    auto low_again = readyInput(1.04, 0.7, 0.5);
    low_again.altitude.value = 0.1;
    postAllInputEvents(runtime, low_again);
    const auto ground_again = runtime.update(1.04);

    EXPECT_EQ(ground_again.state, state_type::Ground);
    EXPECT_FALSE(ground_again.sample_used);
}

TEST(HoverThrustEstimatorRuntimeTest, RawUpdateIsLimitedToConfiguredRate) {
    HoverThrustEstimatorRuntime runtime;

    postAllInputEvents(runtime, readyInput(1.0, 0.8, 0.5));
    const auto first_output = runtime.update(1.0);
    ASSERT_EQ(first_output.state, state_type::Airborne);
    ASSERT_TRUE(first_output.sample_used);
    ASSERT_DOUBLE_EQ(first_output.last_estimate_stamp_sec, 1.0);

    postAllInputEvents(runtime, readyInput(1.05, 0.7, 0.5));
    const auto skipped_output = runtime.update(1.05);
    EXPECT_EQ(skipped_output.state, state_type::Airborne);
    EXPECT_FALSE(skipped_output.sample_used);
    EXPECT_DOUBLE_EQ(skipped_output.last_estimate_stamp_sec, 1.0);

    postAllInputEvents(runtime, readyInput(1.101, 0.7, 0.5));
    const auto updated_output = runtime.update(1.101);
    EXPECT_EQ(updated_output.state, state_type::Airborne);
    EXPECT_TRUE(updated_output.sample_used);
    EXPECT_DOUBLE_EQ(updated_output.last_estimate_stamp_sec, 1.101);
}

}  // namespace
}  // namespace hover_thrust_estimator

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
