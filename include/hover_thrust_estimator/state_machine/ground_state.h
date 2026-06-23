#pragma once

#include "hover_thrust_estimator/common/periodic_gate.h"
#include "hover_thrust_estimator/state_machine/state_adapter.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class GroundState final : public StateAdapter {
   public:
    explicit GroundState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "Ground";
    }

   protected:
    void onEnter() override;
    void onPerform() override;
    void onExit() override;

   private:
    void publishEstimateIfDue();

    HoverThrustEstimatorRuntime& runtime_;
    PeriodicGate publish_gate_;
};

}  // namespace hover_thrust_estimator
