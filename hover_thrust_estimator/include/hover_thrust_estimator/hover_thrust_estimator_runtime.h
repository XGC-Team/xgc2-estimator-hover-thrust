#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <state_machine/adt_machine.hpp>
#include <state_machine/state_machine.hpp>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "hover_thrust_estimator/common/event_types.h"
#include "hover_thrust_estimator/common/periodic_gate.h"
#include "hover_thrust_estimator/common/types.h"
#include "hover_thrust_estimator/hover_thrust_estimator.h"
#include "hover_thrust_estimator/hover_thrust_output_model.h"

namespace hover_thrust_estimator {

struct SelfCheckState {
    static constexpr HoverThrustStateId kStateId = state_type::SelfCheck;
    PeriodicGate publish_gate;
};

struct GroundState {
    static constexpr HoverThrustStateId kStateId = state_type::Ground;
    PeriodicGate publish_gate;
};

struct AirborneState {
    static constexpr HoverThrustStateId kStateId = state_type::Airborne;
    PeriodicGate raw_update_gate;
    PeriodicGate publish_gate;
};

using HoverThrustState = std::variant<SelfCheckState, GroundState, AirborneState>;
using HoverThrustTransition =
    ::state_machine::adt::Transition<SelfCheckState, GroundState, AirborneState>;

inline HoverThrustStateId stateId(const HoverThrustState& state) {
    if (std::holds_alternative<SelfCheckState>(state)) {
        return SelfCheckState::kStateId;
    }
    if (std::holds_alternative<GroundState>(state)) {
        return GroundState::kStateId;
    }
    return AirborneState::kStateId;
}

// Product-local publication identity. It replaces the old state_machine::Event
// shell at the facade boundary without inventing a second publication authority.
struct PublicationEvent {
    uint32_t id{output_event_type::PUBLISH_ESTIMATE};
    double timestamp{0.0};
    uint64_t sequence{0};
};

struct PublishedEstimate {
    PublicationEvent event;
    double stamp_sec{0.0};
    HoverThrustOutput output;
};

// One actual health-state edge. There is deliberately no tick/empty record.
struct TransitionTrace {
    uint64_t sequence{0};
    double stamp_sec{0.0};
    uint32_t event_id{0};
    HoverThrustStateId from_state{state_type::SelfCheck};
    HoverThrustStateId to_state{state_type::SelfCheck};
    int32_t priority{transition_priority::AUTOMATIC};
};

struct HoverThrustTick {
    double now_sec{0.0};
};

struct HoverThrustHealthEdge {
    HoverThrustHealthCondition condition{HoverThrustHealthCondition::kInputUnhealthy};
    uint32_t flags{0};
    double now_sec{0.0};
};

class HoverThrustEstimatorRuntime;
struct HoverThrustSpec;

class HoverThrustEstimatorRuntime {
   public:
    using Config = HoverThrustEstimatorConfig;
    using Sample = HoverThrustSample;
    using Input = HoverThrustInput;
    using Output = HoverThrustOutput;
    using HealthStatus = HoverThrustHealthStatus;
    struct Wake {
        double now_sec{0.0};
        std::optional<double> next_deadline_sec;
    };

    HoverThrustEstimatorRuntime();
    ~HoverThrustEstimatorRuntime();
    HoverThrustEstimatorRuntime(const HoverThrustEstimatorRuntime&) = delete;
    HoverThrustEstimatorRuntime& operator=(const HoverThrustEstimatorRuntime&) = delete;

    void setConfig(const Config& config);
    void reset();
    // Source stamp controls sample period/freshness; now is the caller's
    // processing clock for the input event. No clock or zero-stamp fallback
    // exists in this ROS-free runtime. These methods only record input facts
    // and the owner-local event time; update(now) advances the domain once
    // after the caller's input batch.
    ::state_machine::Status ingestImu(double acceleration_z, double stamp_sec, double now_sec);
    ::state_machine::Status ingestThrust(double normalized_thrust, bool ignored, double stamp_sec,
                                         double now_sec);
    ::state_machine::Status ingestAltitude(double altitude, double stamp_sec, double now_sec);
    // Test/owner batch helper: applies one complete Input and records one
    // event time. Production ROS and embedded callers use ingest* above.
    ::state_machine::Status applyInput(const Input& input, double now_sec);

    // One causal owner turn: pending input health edges, the health evaluation
    // at now, the resulting state's due raw/publication actions, and output
    // drive happen here without an engine transition/tick double pass.
    Output update(double now_sec);
    Output output(double now_sec) const;
    Output snapshotOutput() const {
        return last_output_;
    }
    bool hasPendingInput() const {
        return !pending_input_marks_.empty();
    }
    Wake nextWake(double now_sec) const;

    const std::vector<PublishedEstimate>& publishedEstimates() const {
        return published_estimates_;
    }
    const std::vector<TransitionTrace>& transitionTrace() const {
        return transition_trace_;
    }
    HoverThrustStateId currentState() const;
    uint64_t updateIndex() const {
        return owner_turn_;
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

   private:
    friend struct HoverThrustSpec;
    using Machine = ::state_machine::adt::AdtMachine<HoverThrustSpec>;

    struct PublicationRequest {
        double stamp_sec{0.0};
    };

    void ingestSample(Sample& sample, double value, double stamp_sec, double now_sec);
    void evaluateHealthAt(double now_sec);
    void onEnterState(HoverThrustStateId state);
    void tickState(SelfCheckState& state, double now_sec);
    void tickState(GroundState& state, double now_sec);
    void tickState(AirborneState& state, double now_sec);
    bool updateRawEstimateIfDue(PeriodicGate& gate, double now_sec, uint32_t& flags);
    void publishEstimateIfDue(PeriodicGate& gate, double now_sec);
    void recordStateOutput(HoverThrustStateId state, uint32_t flags, bool sample_used);
    Output makeOutput(HoverThrustStateId state, uint32_t flags, bool sample_used,
                      const HealthStatus& health) const;
    double publishPeriodSec() const {
        return 1.0 / config_.publish_rate_hz;
    }
    double rawPeriodSec() const {
        return 1.0 / config_.raw_update_rate_hz;
    }

    Config config_{};
    HoverThrustEstimator estimator_{};
    std::unique_ptr<Machine> machine_;
    uint32_t flags_{0};
    Input input_{};
    HealthStatus health_{};
    HoverThrustOutputModel output_model_{};
    double last_estimate_stamp_sec_{0.0};
    bool last_sample_used_{false};
    Output last_output_{};
    std::vector<double> pending_input_marks_;
    std::vector<PublicationRequest> publication_requests_;
    std::vector<PublishedEstimate> published_estimates_;
    std::vector<TransitionTrace> transition_trace_;
    uint64_t publication_sequence_{0};
    uint64_t trace_sequence_{0};
    uint64_t owner_turn_{0};
};

// The spec is intentionally thin: it only chooses a health edge target and
// delegates the existing state-local due work to the runtime owner.
struct HoverThrustSpec {
    using State = HoverThrustState;
    using Time = double;

    template <class Active>
    static HoverThrustTransition handle(Active&, const HoverThrustHealthEdge& edge,
                                        HoverThrustEstimatorRuntime&) {
        switch (edge.condition) {
            case HoverThrustHealthCondition::kInputUnhealthy:
                if constexpr (std::is_same_v<Active, SelfCheckState>) {
                    return ::state_machine::adt::Stay{};
                } else {
                    return ::state_machine::adt::Enter<SelfCheckState>{};
                }
            case HoverThrustHealthCondition::kBelowMinAltitude:
                if constexpr (std::is_same_v<Active, GroundState>) {
                    return ::state_machine::adt::Stay{};
                } else {
                    return ::state_machine::adt::Enter<GroundState>{};
                }
            case HoverThrustHealthCondition::kEstimationReady:
                if constexpr (std::is_same_v<Active, AirborneState>) {
                    return ::state_machine::adt::Stay{};
                } else {
                    return ::state_machine::adt::Enter<AirborneState>{};
                }
        }
        return ::state_machine::adt::Stay{};
    }

    template <class Active>
    static HoverThrustTransition handle(Active& active, const HoverThrustTick& tick,
                                        HoverThrustEstimatorRuntime& runtime) {
        runtime.tickState(active, tick.now_sec);
        return ::state_machine::adt::Stay{};
    }
};

}  // namespace hover_thrust_estimator
