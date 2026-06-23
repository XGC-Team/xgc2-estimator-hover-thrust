#pragma once

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
    HoverThrustEstimatorRuntime& runtime_;
};

}  // namespace hover_thrust_estimator
