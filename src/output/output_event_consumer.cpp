#include "hover_thrust_estimator/output/output_event_consumer.h"

#include <ros/ros.h>

#include <utility>

namespace hover_thrust_estimator {

void OutputEventDispatcher::addConsumer(std::unique_ptr<OutputEventConsumer> consumer) {
    if (consumer) {
        consumers_.push_back(std::move(consumer));
    }
}

void OutputEventDispatcher::dispatch(const std::vector<::state_machine::Event>& events) {
    for (const auto& event : events) {
        bool handled = false;
        for (auto& consumer : consumers_) {
            if (consumer->handle(event)) {
                handled = true;
                break;
            }
        }
        if (!handled) {
            ROS_WARN("[HoverThrustOutputDispatcher] Unhandled output event id: %u", event.id);
        }
    }
}

}  // namespace hover_thrust_estimator
