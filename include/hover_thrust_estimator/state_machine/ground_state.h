#pragma once

#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/common/periodic_gate.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class GroundState final : public ::state_machine::State {
   public:
    explicit GroundState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "Ground";
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
