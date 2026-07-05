#pragma once

#include <state_machine/state_machine.hpp>

#include "hover_thrust_estimator/common/periodic_gate.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class AirborneState final : public ::state_machine::State {
   public:
    explicit AirborneState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "Airborne";
    }

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;
    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) override;

   private:
    bool updateRawEstimateIfDue(uint32_t& flags);
    void publishEstimateIfDue(::state_machine::StateContext& ctx);

    HoverThrustEstimatorRuntime& runtime_;
    PeriodicGate raw_update_gate_;
    PeriodicGate publish_gate_;
};

}  // namespace hover_thrust_estimator
