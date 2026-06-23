#pragma once

#include "hover_thrust_estimator/common/periodic_gate.h"
#include "hover_thrust_estimator/state_machine/state_adapter.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class AirborneState final : public StateAdapter {
   public:
    explicit AirborneState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "Airborne";
    }

   protected:
    void onEnter() override;
    void onPerform() override;
    void onExit() override;

   private:
    bool updateRawEstimateIfDue(uint32_t& flags);
    void publishEstimateIfDue();

    HoverThrustEstimatorRuntime& runtime_;
    PeriodicGate raw_update_gate_;
    PeriodicGate publish_gate_;
};

}  // namespace hover_thrust_estimator
