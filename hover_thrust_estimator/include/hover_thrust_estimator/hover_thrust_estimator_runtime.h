#pragma once

#include <memory>
#include <state_machine/state_machine.hpp>
#include <vector>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/types.h"
#include "hover_thrust_estimator/hover_thrust_estimator.h"
#include "hover_thrust_estimator/hover_thrust_output_model.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime {
   public:
    using Config = HoverThrustEstimatorConfig;
    using Sample = HoverThrustSample;
    using Input = HoverThrustInput;
    using Output = HoverThrustOutput;
    using HealthStatus = HoverThrustHealthStatus;
    using DebugTraceRecord = HoverThrustDebugTraceRecord;

    HoverThrustEstimatorRuntime();
    HoverThrustEstimatorRuntime(const HoverThrustEstimatorRuntime&) = delete;
    HoverThrustEstimatorRuntime& operator=(const HoverThrustEstimatorRuntime&) = delete;

    void setConfig(const Config& config);
    void reset();
    ::state_machine::Status postInputEvent(::state_machine::Event event, const Input& input);
    Output update(double now_sec);
    Output output(double now_sec) const;
    Output snapshotOutput() const {
        return last_output_;
    }
    Output refreshOutputSnapshot();
    ::state_machine::StateMachine& getStateMachine() {
        return *machine_;
    }
    const ::state_machine::StateMachine& getStateMachine() const {
        return *machine_;
    }

    const Config& config() const {
        return config_;
    }
    const Input& input() const {
        return input_;
    }
    const HealthStatus& health() const {
        return health_;
    }
    void setHealth(const HealthStatus& health) {
        health_ = health;
    }
    HoverThrustStateId currentState() const {
        return state_;
    }
    void enterState(HoverThrustStateId state);
    HoverThrustEstimator& estimator() {
        return estimator_;
    }
    const HoverThrustEstimator& estimator() const {
        return estimator_;
    }
    HoverThrustOutputModel& outputModel() {
        return output_model_;
    }
    const HoverThrustOutputModel& outputModel() const {
        return output_model_;
    }
    void setLastEstimateStamp(double stamp_sec) {
        last_estimate_stamp_sec_ = stamp_sec;
    }
    Output recordStateOutput(HoverThrustStateId state, uint32_t flags, bool sample_used);
    double currentTime() const {
        return current_time_sec_;
    }
    const std::vector<DebugTraceRecord>& debugTraceRecords() const {
        return debug_trace_records_;
    }
    bool hasDebugTrace() const {
        return !debug_trace_records_.empty();
    }
    std::vector<::state_machine::Event> debugOutputEvents(double now_sec) const;

   private:
    void setupMachine();
    void applyInputEvent(const Input& input);
    void appendDebugTrace(HoverThrustDebugTracePhase phase,
                          const std::vector<::state_machine::EventTraceRecord>& trace);
    static bool isDebugTraceRecordInteresting(const ::state_machine::EventTraceRecord& record);
    Output makeOutput(HoverThrustStateId state, uint32_t flags, bool sample_used,
                      const HealthStatus& health) const;

    Config config_{};
    HoverThrustEstimator estimator_{};
    std::unique_ptr<state_machine::StateMachine> machine_;
    HoverThrustStateId state_{state_type::SelfCheck};
    uint32_t flags_{0};
    Input input_{};
    HealthStatus health_{};
    HoverThrustOutputModel output_model_{};
    double current_time_sec_{0.0};
    double last_estimate_stamp_sec_{0.0};
    bool last_sample_used_{false};
    Output last_output_{};
    std::vector<DebugTraceRecord> debug_trace_records_;
};

}  // namespace hover_thrust_estimator
