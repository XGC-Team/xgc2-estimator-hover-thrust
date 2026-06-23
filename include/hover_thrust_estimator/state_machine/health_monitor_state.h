#pragma once

#include <state_machine/state_machine.hpp>

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class HealthMonitorState final : public ::state_machine::State {
   public:
    explicit HealthMonitorState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "HealthMonitor";
    }

   protected:
    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) override;

   private:
    HoverThrustEstimatorRuntime& runtime_;
};

}  // namespace hover_thrust_estimator
