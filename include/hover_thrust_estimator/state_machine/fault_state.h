#pragma once

#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/common/periodic_gate.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class FaultState final : public ::state_machine::State {
   public:
    explicit FaultState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "Fault";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    void publishEstimateIfDue(::state_machine::StateContext& ctx);

    HoverThrustEstimatorRuntime& runtime_;
    PeriodicGate publish_gate_;
};

}  // namespace hover_thrust_estimator
