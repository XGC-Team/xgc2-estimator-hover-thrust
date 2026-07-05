#pragma once

#include <cstdint>

namespace hover_thrust_estimator {

namespace state_type {
constexpr uint32_t HealthMonitor = 1;
constexpr uint32_t SelfCheck = 10;
constexpr uint32_t Ground = 11;
constexpr uint32_t Airborne = 12;
}  // namespace state_type

namespace region_type {
constexpr uint32_t HEALTH = 1;
constexpr uint32_t ESTIMATION = 2;
}  // namespace region_type

namespace event_type {
constexpr uint32_t INPUT_IMU_UPDATED = 100;
constexpr uint32_t INPUT_THRUST_UPDATED = 101;
constexpr uint32_t INPUT_ALTITUDE_UPDATED = 102;
constexpr uint32_t HEALTH_INPUT_UNHEALTHY = 200;
constexpr uint32_t HEALTH_BELOW_MIN_ALTITUDE = 201;
constexpr uint32_t HEALTH_ESTIMATION_READY = 202;
}  // namespace event_type

namespace output_event_type {
constexpr uint32_t PUBLISH_ESTIMATE = 1000;
constexpr uint32_t PUBLISH_DEBUG_TRACE = 1001;
}  // namespace output_event_type

namespace transition_priority {
constexpr int AUTOMATIC = 10;
}  // namespace transition_priority

}  // namespace hover_thrust_estimator
