#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <xgc2_math/utils/sample_timing.hpp>

#include "hover_thrust_estimator/common/config_utils.h"
#include "hover_thrust_estimator/common/health_checks.h"

namespace hover_thrust_estimator {
namespace {

uint32_t eventIdForCondition(HoverThrustHealthCondition condition) {
    switch (condition) {
        case HoverThrustHealthCondition::kInputUnhealthy:
            return event_type::HEALTH_INPUT_UNHEALTHY;
        case HoverThrustHealthCondition::kBelowMinAltitude:
            return event_type::HEALTH_BELOW_MIN_ALTITUDE;
        case HoverThrustHealthCondition::kEstimationReady:
            return event_type::HEALTH_ESTIMATION_READY;
    }
    return event_type::HEALTH_INPUT_UNHEALTHY;
}

HoverThrustStateId transitionTarget(const HoverThrustTransition& transition) {
    return std::visit(
        [](const auto& outcome) -> HoverThrustStateId {
            using Outcome = std::decay_t<decltype(outcome)>;
            if constexpr (std::is_same_v<Outcome, ::state_machine::adt::Stay>) {
                return 0;
            } else {
                return std::decay_t<decltype(outcome.value)>::kStateId;
            }
        },
        transition);
}

}  // namespace

HoverThrustEstimatorRuntime::HoverThrustEstimatorRuntime() {
    config_utils::normalizeConfig(config_);
    reset();
}

HoverThrustEstimatorRuntime::~HoverThrustEstimatorRuntime() = default;

void HoverThrustEstimatorRuntime::setConfig(const Config& config) {
    config_ = config;
    config_utils::normalizeConfig(config_);
    reset();
}

void HoverThrustEstimatorRuntime::reset() {
    estimator_.setConfig(HoverThrustEstimator::Config{config_.rho2, config_.min_hover_thrust,
                                                      config_.max_hover_thrust, false, 0.0});
    estimator_.reset(config_.gravity, config_.initial_hover_thrust);
    input_ = Input{};
    health_ = HealthStatus{};
    output_model_.reset(config_);
    last_estimate_stamp_sec_ = 0.0;
    last_sample_used_ = false;
    flags_ = 0;
    machine_ = std::make_unique<Machine>(SelfCheckState{}, 0.0);
    pending_input_marks_.clear();
    published_estimates_.clear();
    transition_trace_.clear();
    trace_sequence_ = 0;
    owner_turn_ = 0;
    last_output_ = makeOutput(currentState(), flags_, false, health_);
}

void HoverThrustEstimatorRuntime::ingestSample(Sample& sample, double value, double stamp_sec,
                                               double now_sec) {
    sample.period_sec =
        sample.received && std::isfinite(sample.stamp_sec) && std::isfinite(stamp_sec)
            ? stamp_sec - sample.stamp_sec
            : 0.0;
    sample.value = value;
    sample.stamp_sec = stamp_sec;
    sample.received = true;
    sample.finite = std::isfinite(value);
    input_.batch_time_jump = input_.batch_time_jump ||
                             xgc2_math::sampleTimeJumped(now_sec, stamp_sec, sample.period_sec);
    pending_input_marks_.push_back(now_sec);
}

void HoverThrustEstimatorRuntime::ingestImu(double acceleration_z, double stamp_sec,
                                            double now_sec) {
    ingestSample(input_.imu_acc_z, acceleration_z, stamp_sec, now_sec);
}

void HoverThrustEstimatorRuntime::ingestThrust(double normalized_thrust, bool ignored,
                                               double stamp_sec, double now_sec) {
    input_.thrust_ignored = ignored;
    ingestSample(input_.normalized_thrust, normalized_thrust, stamp_sec, now_sec);
}

void HoverThrustEstimatorRuntime::ingestAltitude(double altitude, double stamp_sec,
                                                 double now_sec) {
    ingestSample(input_.altitude, altitude, stamp_sec, now_sec);
}

void HoverThrustEstimatorRuntime::applyInput(const Input& input, double now_sec) {
    input_ = input;
    pending_input_marks_.push_back(now_sec);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::update(double now_sec) {
    published_estimates_.clear();
    transition_trace_.clear();

    // Pending input events are evaluated in arrival order at their own event
    // clock, exactly as the old health monitor did for posted input events.
    for (const double mark_sec : pending_input_marks_) {
        evaluateHealthAt(mark_sec);
    }
    pending_input_marks_.clear();

    // The idle tick still evaluates health at the owner clock. A condition edge
    // now transitions in this same turn instead of leaving a one-turn stale
    // output behind.
    evaluateHealthAt(now_sec);

    auto tick_transition = machine_->dispatch(HoverThrustTick{now_sec}, *this);
    machine_->apply(std::move(tick_transition), now_sec);

    input_.batch_time_jump = false;
    ++owner_turn_;
    return last_output_;
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::output(double now_sec) const {
    (void)now_sec;
    return makeOutput(currentState(), flags_, false, health_);
}

HoverThrustEstimatorRuntime::Wake HoverThrustEstimatorRuntime::nextWake(double now_sec) const {
    Wake wake;
    wake.now_sec = now_sec;

    std::optional<double> next;
    auto consider = [&next](std::optional<double> deadline) {
        if (deadline && std::isfinite(*deadline) && (!next || *deadline < *next)) {
            next = *deadline;
        }
    };

    if (!pending_input_marks_.empty()) {
        next = now_sec;
    }

    std::visit(
        [&](const auto& state) {
            using State = std::decay_t<decltype(state)>;
            if constexpr (std::is_same_v<State, AirborneState>) {
                consider(state.raw_update_gate.nextDue(now_sec, rawPeriodSec()));
            }
            consider(state.publish_gate.nextDue(now_sec, publishPeriodSec()));
        },
        machine_->state());

    // sampleStale() is strict: it becomes true after stamp + timeout.
    const Sample* samples[] = {&input_.imu_acc_z, &input_.normalized_thrust, &input_.altitude};
    for (const Sample* sample : samples) {
        if (!sample->received || !std::isfinite(sample->stamp_sec)) {
            continue;
        }
        const double boundary = std::nextafter(sample->stamp_sec + config_.sample_timeout,
                                               std::numeric_limits<double>::infinity());
        // Once the timeout is already in the past, health has been (or is about
        // to be) evaluated at now; keeping that past deadline would busy-loop
        // the node until the next sample. Only future timeout edges are wakes.
        if (std::isfinite(boundary) && boundary > now_sec) {
            consider(boundary);
        }
    }

    wake.next_deadline_sec = next;
    return wake;
}

HoverThrustStateId HoverThrustEstimatorRuntime::currentState() const {
    return stateId(machine_->state());
}

void HoverThrustEstimatorRuntime::evaluateHealthAt(double now_sec) {
    const HoverThrustHealthCondition previous_condition = health_.condition;
    const auto health = health_checks::classify(input_, config_, now_sec);
    health_ = health;
    if (health.condition == previous_condition) {
        return;
    }

    const HoverThrustStateId from_state = currentState();
    auto transition =
        machine_->dispatch(HoverThrustHealthEdge{health.condition, health.flags, now_sec}, *this);
    const HoverThrustStateId to_state = transitionTarget(transition);
    if (to_state == 0) {
        return;
    }

    onEnterState(to_state);
    machine_->apply(std::move(transition), now_sec);
    transition_trace_.push_back(TransitionTrace{++trace_sequence_, now_sec,
                                                eventIdForCondition(health.condition), from_state,
                                                to_state, transition_priority::AUTOMATIC});
}

void HoverThrustEstimatorRuntime::onEnterState(HoverThrustStateId state) {
    switch (state) {
        case state_type::SelfCheck:
            output_model_.setTarget(config_.initial_hover_thrust);
            break;
        case state_type::Ground:
            output_model_.freezeTarget();
            break;
        case state_type::Airborne:
            break;
        default:
            break;
    }
}

void HoverThrustEstimatorRuntime::tickState(SelfCheckState& state, double now_sec) {
    if (health_.condition != HoverThrustHealthCondition::kInputUnhealthy) {
        return;
    }
    recordStateOutput(state_type::SelfCheck, health_.flags, false);
    publishEstimateIfDue(state.publish_gate, now_sec);
}

void HoverThrustEstimatorRuntime::tickState(GroundState& state, double now_sec) {
    if (health_.condition != HoverThrustHealthCondition::kBelowMinAltitude) {
        return;
    }
    recordStateOutput(state_type::Ground, health_.flags, false);
    publishEstimateIfDue(state.publish_gate, now_sec);
}

void HoverThrustEstimatorRuntime::tickState(AirborneState& state, double now_sec) {
    if (health_.condition != HoverThrustHealthCondition::kEstimationReady) {
        return;
    }

    uint32_t flags = health_.flags;
    const bool sample_used = updateRawEstimateIfDue(state.raw_update_gate, now_sec, flags);
    recordStateOutput(state_type::Airborne, flags, sample_used);
    publishEstimateIfDue(state.publish_gate, now_sec);
}

bool HoverThrustEstimatorRuntime::updateRawEstimateIfDue(PeriodicGate& gate, double now_sec,
                                                         uint32_t& flags) {
    if (!gate.due(now_sec, rawPeriodSec())) {
        return false;
    }
    if (!health_.ready) {
        flags |= HoverThrustRuntimeFlag::kRawEstimateStale;
        return false;
    }

    const auto& input = input_;
    const bool sample_used = estimator_.update(input.imu_acc_z.value, input.normalized_thrust.value,
                                               input.normalized_thrust.stamp_sec);
    if (!sample_used) {
        flags |= HoverThrustRuntimeFlag::kEstimatorRejected;
        return false;
    }

    output_model_.setRaw(estimator_.rawEstimate());
    output_model_.setTarget(estimator_.rawEstimate());
    last_estimate_stamp_sec_ = input.normalized_thrust.stamp_sec;
    return true;
}

void HoverThrustEstimatorRuntime::publishEstimateIfDue(PeriodicGate& gate, double now_sec) {
    if (!gate.due(now_sec, publishPeriodSec())) {
        return;
    }
    output_model_.driveTowardTarget(now_sec);
    last_output_ = makeOutput(currentState(), flags_, last_sample_used_, health_);
    published_estimates_.push_back(PublishedEstimate{now_sec, last_output_});
}

void HoverThrustEstimatorRuntime::recordStateOutput(HoverThrustStateId state, uint32_t flags,
                                                    bool sample_used) {
    flags_ = flags;
    last_sample_used_ = sample_used;
    last_output_ = makeOutput(state, flags_, sample_used, health_);
}

HoverThrustEstimatorRuntime::Output HoverThrustEstimatorRuntime::makeOutput(
    HoverThrustStateId state, uint32_t flags, bool sample_used, const HealthStatus& health) const {
    Output output;
    output.state = state;
    output.flags = flags;
    output.hover_thrust = output_model_.hoverThrust();
    output.raw_hover_thrust = output_model_.raw();
    output.initial_hover_thrust = config_.initial_hover_thrust;
    output.thrust_to_acceleration = output.hover_thrust > estimator_limits::kMinimumNormalizedThrust
                                        ? config_.gravity / output.hover_thrust
                                        : config_.gravity / config_.initial_hover_thrust;
    output.sample_used = sample_used;
    output.source_stamp_sec = health.source_stamp_sec;
    output.last_estimate_stamp_sec = last_estimate_stamp_sec_;
    return output;
}

}  // namespace hover_thrust_estimator
