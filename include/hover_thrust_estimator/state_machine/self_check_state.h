#pragma once

#include "hover_thrust_estimator/state_machine/state_adapter.h"

namespace hover_thrust_estimator {

class HoverThrustEstimatorRuntime;

class SelfCheckState final : public StateAdapter {
   public:
    explicit SelfCheckState(HoverThrustEstimatorRuntime& runtime);

    std::string name() const override {
        return "SelfCheck";
    }

   protected:
    void onEnter() override;
    void onPerform() override;
    void onExit() override;

   private:
    HoverThrustEstimatorRuntime& runtime_;
};

}  // namespace hover_thrust_estimator
