#pragma once

#include <chrono>
#include <state_machine/runtime/steady_timer.hpp>
#include <state_machine/state_machine.hpp>
#include <utility>

namespace hover_thrust_estimator {

template <typename Clock = std::chrono::steady_clock,
          typename Duration = std::chrono::duration<double>>
using Timer = ::state_machine::runtime::Timer<Clock, Duration>;

class StateAdapter : public ::state_machine::State {
   public:
    explicit StateAdapter(::state_machine::StateId state_id) : state_id_(state_id) {}
    ~StateAdapter() override = default;

   protected:
    ::state_machine::ActionResult onEnter(::state_machine::StateContext& ctx) final {
        ScopedContext scope(*this, ctx);
        onEnter();
        return {};
    }

    ::state_machine::ActionResult onTick(::state_machine::StateContext& ctx) final {
        ScopedContext scope(*this, ctx);
        onPerform();
        return {};
    }

    ::state_machine::ActionResult onExit(::state_machine::StateContext& ctx) final {
        ScopedContext scope(*this, ctx);
        onExit();
        return {};
    }

    ::state_machine::Status postInternalEvent(::state_machine::Event event) const {
        if (!current_context_) {
            return ::state_machine::Status::error(::state_machine::ErrorCode::kInvalidLifecycle,
                                                  "state context is not active");
        }
        return current_context_->postInternalEvent(std::move(event));
    }

    ::state_machine::Status emitOutputEvent(::state_machine::Event event) const {
        if (!current_context_) {
            return ::state_machine::Status::error(::state_machine::ErrorCode::kInvalidLifecycle,
                                                  "state context is not active");
        }
        event.category = ::state_machine::EventCategory::kOutput;
        return current_context_->emitOutput(std::move(event));
    }

    ::state_machine::Status emitOutputEvent(::state_machine::EventId event_id,
                                            double timestamp) const {
        return emitOutputEvent(
            ::state_machine::Event(event_id, ::state_machine::EventTimestamp{timestamp}));
    }

    virtual void onEnter() = 0;
    virtual void onPerform() {}
    virtual void onExit() = 0;

   private:
    class ScopedContext {
       public:
        ScopedContext(StateAdapter& state, ::state_machine::StateContext& ctx) : state_(state) {
            state_.current_context_ = &ctx;
        }
        ~ScopedContext() {
            state_.current_context_ = nullptr;
        }
        ScopedContext(const ScopedContext&) = delete;
        ScopedContext& operator=(const ScopedContext&) = delete;

       private:
        StateAdapter& state_;
    };

    ::state_machine::StateId state_id_{0};
    ::state_machine::StateContext* current_context_{nullptr};
};

}  // namespace hover_thrust_estimator
