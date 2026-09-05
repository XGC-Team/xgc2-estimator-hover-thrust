#include <gtest/gtest.h>

#include <limits>

#include "hover_thrust_estimator/hover_thrust_estimator.h"

namespace hover_thrust_estimator {
namespace {

constexpr double kGravity = 9.8066;

void expectSameEstimate(const HoverThrustEstimator& actual, const HoverThrustEstimator& expected) {
    EXPECT_DOUBLE_EQ(actual.estimate(), expected.estimate());
    EXPECT_DOUBLE_EQ(actual.rawEstimate(), expected.rawEstimate());
    EXPECT_DOUBLE_EQ(actual.lastTimeSec(), expected.lastTimeSec());
    EXPECT_EQ(actual.valid(), expected.valid());
}

TEST(HoverThrustRejection, FirstRejectedCandidateDoesNotPoisonNextNormalSample) {
    HoverThrustEstimator actual;
    HoverThrustEstimator expected;
    actual.reset(kGravity, 0.3);
    expected.reset(kGravity, 0.3);

    ASSERT_FALSE(actual.update(-1000.0, 0.3, 1.0));
    expectSameEstimate(actual, expected);
    ASSERT_TRUE(expected.update(kGravity, 0.3, 1.01));
    ASSERT_TRUE(actual.update(kGravity, 0.3, 1.01));
    expectSameEstimate(actual, expected);
}

TEST(HoverThrustRejection, RejectionAfterAcceptancePreservesRlsAndFilterState) {
    for (const bool filter_enabled : {false, true}) {
        SCOPED_TRACE(filter_enabled);
        HoverThrustEstimator::Config config;
        config.filter_enabled = filter_enabled;
        HoverThrustEstimator actual;
        HoverThrustEstimator expected;
        actual.setConfig(config);
        expected.setConfig(config);
        actual.reset(kGravity, 0.3);
        expected.reset(kGravity, 0.3);
        ASSERT_TRUE(actual.update(kGravity, 0.4, 1.0));
        ASSERT_TRUE(expected.update(kGravity, 0.4, 1.0));

        ASSERT_FALSE(actual.update(-1000.0, 0.3, 1.02));
        expectSameEstimate(actual, expected);
        for (int i = 1; i <= 20; ++i) {
            const double stamp = 1.02 + 0.02 * static_cast<double>(i);
            ASSERT_TRUE(actual.update(kGravity, 0.35, stamp));
            ASSERT_TRUE(expected.update(kGravity, 0.35, stamp));
            expectSameEstimate(actual, expected);
        }
    }
}

TEST(HoverThrustRejection, RepeatedRejectionsDoNotConsumeTimestampOrCovariance) {
    HoverThrustEstimator actual;
    HoverThrustEstimator expected;
    actual.reset(kGravity, 0.3);
    expected.reset(kGravity, 0.3);
    ASSERT_TRUE(actual.update(kGravity, 0.3, 1.0));
    ASSERT_TRUE(expected.update(kGravity, 0.3, 1.0));
    for (int i = 0; i < 20; ++i) {
        ASSERT_FALSE(actual.update(-1000.0, 0.3, 2.0));
        expectSameEstimate(actual, expected);
    }
    ASSERT_TRUE(actual.update(kGravity, 0.4, 2.0));
    ASSERT_TRUE(expected.update(kGravity, 0.4, 2.0));
    expectSameEstimate(actual, expected);
}

TEST(HoverThrustRejection, InvalidAndDuplicateInputsStillHoldAcceptedState) {
    HoverThrustEstimator actual;
    HoverThrustEstimator expected;
    actual.reset(kGravity, 0.3);
    expected.reset(kGravity, 0.3);
    ASSERT_TRUE(actual.update(kGravity, 0.3, 1.0));
    ASSERT_TRUE(expected.update(kGravity, 0.3, 1.0));
    ASSERT_FALSE(actual.update(std::numeric_limits<double>::quiet_NaN(), 0.3, 2.0));
    ASSERT_FALSE(actual.update(kGravity, 0.0, 2.0));
    ASSERT_FALSE(actual.update(kGravity, 1.1, 2.0));
    ASSERT_FALSE(actual.update(kGravity, 0.3, 1.0));
    expectSameEstimate(actual, expected);
    ASSERT_TRUE(actual.update(kGravity, 0.4, 2.0));
    ASSERT_TRUE(expected.update(kGravity, 0.4, 2.0));
    expectSameEstimate(actual, expected);
}

}  // namespace
}  // namespace hover_thrust_estimator
