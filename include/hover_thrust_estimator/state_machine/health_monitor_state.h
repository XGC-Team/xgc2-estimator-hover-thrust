#pragma once

#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hover_thrust_estimator {

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "HealthMonitor";
    }

   protected:
    ::state_machine::ActionResult onEvent(::state_machine::StateContext& ctx,
                                          const ::state_machine::Event& event) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    using HealthStatus = HoverThrustEstimatorRuntime::HealthStatus;

    void evaluateAndPostTransition(::state_machine::StateContext& ctx, double now_sec) const;
    HealthStatus classify(double now_sec) const;
    bool sampleStale(const HoverThrustEstimatorRuntime::Sample& sample, double now_sec) const;
    bool sampleRateLow(const HoverThrustEstimatorRuntime::Sample& sample) const;
    bool sampleTimeJumped(const HoverThrustEstimatorRuntime::Sample& sample, double now_sec) const;
    static ::state_machine::EventId transitionEventFor(HoverThrustRuntimeState state);

    HoverThrustEstimatorRuntime& runtime_;
};

}  // namespace hover_thrust_estimator
