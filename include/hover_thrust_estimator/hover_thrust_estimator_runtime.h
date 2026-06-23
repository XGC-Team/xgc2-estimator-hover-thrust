#pragma once

#include <cstdint>
#include <memory>
#include <state_machine/state_machine.hpp>
#include <string>

#include "hover_thrust_estimator/hover_thrust_estimator.h"
#include "xgc2_observer/slew_rate_limiter.hpp"

namespace hover_thrust_estimator {

enum class HoverThrustRuntimeState : uint8_t {
    kSelfCheck = 0,
    kGround = 1,
    kAirborne = 2,
    kFault = 9,
};

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
    kDegraded = 1u << 14,
    kGroundHold = 1u << 15,
    kRawEstimateStale = 1u << 16,
};

enum class HoverThrustInputEvent : uint8_t {
    kImuUpdated = 0,
    kThrustUpdated = 1,
    kAltitudeUpdated = 2,
};

enum class HoverThrustOutputEvent : uint8_t {
    kPublishEstimate = 0,
    kUpdateRawEstimate = 1,
};

class HoverThrustEstimatorRuntime {
   public:
    struct Config {
        double gravity{estimator_limits::kDefaultGravity};
        double initial_hover_thrust{0.5};
        double rho2{0.998};
        double min_hover_thrust{0.05};
        double max_hover_thrust{0.95};
        double min_altitude{0.5};
        double sample_timeout{0.2};
        bool filter_enabled{false};
        double filter_cutoff_hz{2.0};
        double output_slew_rate{0.1};
        double input_rate_low_hz{5.0};
    };

    struct Sample {
        double value{0.0};
        double stamp_sec{0.0};
        double period_sec{0.0};
        bool received{false};
        bool finite{false};
    };

    struct Input {
        double now_sec{0.0};
        Sample imu_acc_z;
        Sample normalized_thrust;
        Sample altitude;
        bool thrust_ignored{true};
    };

    struct Output {
        HoverThrustRuntimeState state{HoverThrustRuntimeState::kSelfCheck};
        uint32_t flags{0};
        double hover_thrust{0.5};
        double raw_hover_thrust{0.5};
        double initial_hover_thrust{0.5};
        double thrust_to_acceleration{estimator_limits::kDefaultGravity / 0.5};
        bool estimator_valid{false};
        bool estimate_valid{false};
        bool degraded{true};
        bool sample_used{false};
        double source_stamp_sec{0.0};
        double last_estimate_stamp_sec{0.0};
    };

    HoverThrustEstimatorRuntime();
    HoverThrustEstimatorRuntime(const HoverThrustEstimatorRuntime&) = delete;
    HoverThrustEstimatorRuntime& operator=(const HoverThrustEstimatorRuntime&) = delete;

    void setConfig(const Config& config);
    void reset();
    void postInputEvent(HoverThrustInputEvent event, const Input& input);
    Output update(HoverThrustOutputEvent event, double now_sec);
    Output output(double now_sec) const;

   private:
    struct Classification {
        HoverThrustRuntimeState state{HoverThrustRuntimeState::kSelfCheck};
        uint32_t flags{0};
        bool ready{false};
        bool degraded{true};
        bool estimate_valid{false};
        double source_stamp_sec{0.0};
    };

    static std::string stateName(HoverThrustRuntimeState state);
    static state_machine::StateId stateId(HoverThrustRuntimeState state);
    static state_machine::EventId eventId(HoverThrustRuntimeState state);

    void normalizeConfig();
    void setupMachine();
    void transitionTo(HoverThrustRuntimeState state);
    Classification classify(double now_sec) const;
    bool sampleStale(const Sample& sample, double now_sec) const;
    bool sampleRateLow(const Sample& sample) const;
    bool sampleTimeJumped(const Sample& sample, double now_sec) const;
    Output makeOutput(HoverThrustRuntimeState state, uint32_t flags, bool sample_used,
                      const Classification& classification, double output_stamp_sec) const;

    Config config_{};
    HoverThrustEstimator estimator_{};
    std::unique_ptr<state_machine::StateMachine> machine_;
    HoverThrustRuntimeState state_{HoverThrustRuntimeState::kSelfCheck};
    uint32_t flags_{0};
    Input input_{};
    xgc2_observer::SlewRateLimiter output_slew_limiter_{};
    double hover_thrust_output_{0.5};
    double raw_hover_thrust_output_{0.5};
    double last_estimate_stamp_sec_{0.0};
};

}  // namespace hover_thrust_estimator
