#pragma once

#include <cstdint>
#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/hover_thrust_estimator.h"

namespace hover_thrust_estimator {

enum HoverThrustRuntimeFlag : uint32_t {
    kImuMissing = 1u << 0,
    kThrustMissing = 1u << 1,
    kAltitudeMissing = 1u << 2,
    kImuStale = 1u << 3,
    kThrustStale = 1u << 4,
    kAltitudeStale = 1u << 5,
    kImuInvalid = 1u << 6,
    kThrustInvalid = 1u << 7,
    kAltitudeInvalid = 1u << 8,
    kBelowMinAltitude = 1u << 9,
    kEstimatorRejected = 1u << 10,
    kStateMachineFault = 1u << 11,
    kInputRateLow = 1u << 12,
    kTimeJump = 1u << 13,
    kGroundHold = 1u << 15,
    kRawEstimateStale = 1u << 16,
};

using HoverThrustStateId = ::state_machine::StateId;

struct HoverThrustEstimatorConfig {
    double gravity{estimator_limits::kDefaultGravity};
    double initial_hover_thrust{0.3};
    double rho2{0.998};
    double min_hover_thrust{0.15};
    double max_hover_thrust{0.85};
    double min_altitude{0.5};
    double sample_timeout{0.2};
    bool filter_enabled{true};
    double filter_cutoff_hz{2.0};
    double input_rate_low_hz{5.0};
    double publish_rate_hz{100.0};
    double raw_update_rate_hz{10.0};
};

struct HoverThrustSample {
    double value{0.0};
    double stamp_sec{0.0};
    double period_sec{0.0};
    bool received{false};
    bool finite{false};
};

struct HoverThrustInput {
    double now_sec{0.0};
    HoverThrustSample imu_acc_z;
    HoverThrustSample normalized_thrust;
    HoverThrustSample altitude;
    bool thrust_ignored{true};
};

struct HoverThrustOutput {
    HoverThrustStateId state{state_type::SelfCheck};
    uint32_t flags{0};
    double hover_thrust{0.3};
    double raw_hover_thrust{0.3};
    double initial_hover_thrust{0.3};
    double thrust_to_acceleration{estimator_limits::kDefaultGravity / 0.3};
    bool sample_used{false};
    double source_stamp_sec{0.0};
    double last_estimate_stamp_sec{0.0};
};

struct HoverThrustHealthStatus {
    HoverThrustStateId state{state_type::SelfCheck};
    ::state_machine::EventId transition_event{event_type::HEALTH_TO_SELF_CHECK};
    uint32_t flags{0};
    bool ready{false};
    double source_stamp_sec{0.0};
};

}  // namespace hover_thrust_estimator
