#pragma once

#include <algorithm>
#include <cmath>

#include "hover_thrust_estimator/common/config_utils.h"
#include "hover_thrust_estimator/common/types.h"
#include "xgc2_observer/exponential_filter.hpp"

namespace hover_thrust_estimator {

class HoverThrustOutputModel {
   public:
    void reset(const HoverThrustEstimatorConfig& config) {
        config_ = config_utils::normalizeConfig(config);
        target_hover_thrust_ = config_.initial_hover_thrust;
        hover_thrust_ = config_.initial_hover_thrust;
        raw_hover_thrust_ = config_.initial_hover_thrust;
        last_update_stamp_sec_ = 0.0;
        filter_.reset(config_.filter_enabled ? config_.filter_cutoff_hz : 0.0, hover_thrust_);
    }

    void setTarget(double hover_thrust) {
        target_hover_thrust_ =
            std::clamp(config_utils::finiteOrDefault(hover_thrust, config_.initial_hover_thrust),
                       config_.min_hover_thrust, config_.max_hover_thrust);
    }

    double target() const {
        return target_hover_thrust_;
    }

    void setRaw(double hover_thrust) {
        raw_hover_thrust_ =
            std::clamp(config_utils::finiteOrDefault(hover_thrust, config_.initial_hover_thrust),
                       config_.min_hover_thrust, config_.max_hover_thrust);
    }

    double raw() const {
        return raw_hover_thrust_;
    }

    double hoverThrust() const {
        return hover_thrust_;
    }

    void driveTowardTarget(double current_time_sec) {
        driveToward(target_hover_thrust_, current_time_sec);
    }

    void driveToward(double target_hover_thrust, double current_time_sec) {
        setTarget(target_hover_thrust);
        hover_thrust_ =
            std::clamp(filter_.filter(target_hover_thrust_, outputDeltaTime(current_time_sec)),
                       config_.min_hover_thrust, config_.max_hover_thrust);
        last_update_stamp_sec_ = current_time_sec;
    }

    void freezeTarget() {
        target_hover_thrust_ =
            std::clamp(target_hover_thrust_, config_.min_hover_thrust, config_.max_hover_thrust);
    }

   private:
    double outputDeltaTime(double current_time_sec) const {
        if (!std::isfinite(current_time_sec)) {
            return 0.0;
        }
        if (!std::isfinite(last_update_stamp_sec_) || last_update_stamp_sec_ <= 0.0) {
            return std::max(0.0, current_time_sec);
        }
        if (current_time_sec + 0.05 < last_update_stamp_sec_) {
            return 0.0;
        }
        return std::max(0.0, current_time_sec - last_update_stamp_sec_);
    }

    HoverThrustEstimatorConfig config_{};
    xgc2_observer::ExponentialLowPass filter_{};
    double target_hover_thrust_{0.3};
    double hover_thrust_{0.3};
    double raw_hover_thrust_{0.3};
    double last_update_stamp_sec_{0.0};
};

}  // namespace hover_thrust_estimator
