// est-hover-thrust: the hover-thrust estimator as an xgc_rt plugin.
//
// Domain-owned thin native facade. It links the SAME exported runtime as the
// ROS facade; no estimator or state policy is copied into this adapter. Runtime has
// its own domain state machine (HealthMonitor region + SelfCheck -> Ground <->
// Airborne) on libxgc2-state-machine. Only the ROS input producer and output
// consumer are replaced, by ports:
//
//   in  imu              xgc.imu/1              (accel z)
//   in  attitude_target  xgc.attitude_target/1  (normalized thrust, ignore mask)
//   in  pose             xgc.pose/1             (altitude = position z)
//   out hover_thrust     xgc.hover_thrust/1     (on each PUBLISH_ESTIMATE)
//
// Scheduling: trigger `both`. Port FIFO heads are merged by source stamp;
// samples within a port are NEVER re-sorted. Shared runtime ingestion observes
// source clock regressions. On a round with no input the runtime is updated so its
// timeouts see time pass. The ROS node polled at 1 kHz to get the same.
//
// Config (TOML, all optional): gravity, initial_hover_thrust, rho2,
// min_hover_thrust, max_hover_thrust, min_altitude, sample_timeout,
// filter_enabled, filter_cutoff_hz, input_rate_low_hz, publish_rate_hz,
// raw_update_rate_hz, time_source = "session" | "input".
// time_source "input" drives the runtime clock from sample stamps only (for
// replay and deterministic tests); "session" (default) uses Session time.

#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <exception>
#include <string>
#include <vector>

#include "flat_config.hpp"
#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"
#include "hover_thrust_estimator/native/hover_thrust_wire.h"
#include "xgc_rt.h"

namespace {

namespace hte = hover_thrust_estimator;
namespace sm = state_machine;

enum Port : uint32_t {
    kImu = 0,
    kAttitudeTarget = 1,
    kPose = 2,
    kHoverThrust = 3,
    kAttitudeTargetFull = 4
};

struct Pending {
    double stamp;
    uint32_t port;
    double value;
    bool ignore_thrust;
};

struct EstHoverThrust {
    const xgc_host_api* host;
    hte::HoverThrustEstimatorRuntime runtime;
    bool input_time{false};
    std::array<std::deque<Pending>, 5> pending;

    void log(xgc_log_level level, const std::string& message) const {
        host->log(host->host, level, message.c_str());
    }

    // Transport supplies source samples and the explicit event clock. The
    // runtime owns period/finite bookkeeping and domain input events.
    bool apply(const Pending& s) {
        const auto status =
            s.port == kImu ? runtime.ingestImu(s.value, s.stamp, s.stamp)
                           : s.port == kAttitudeTarget
                                 ? runtime.ingestThrust(s.value, s.ignore_thrust, s.stamp, s.stamp)
                                 : runtime.ingestAltitude(s.value, s.stamp, s.stamp);
        if (!status.ok())
            log(XGC_LOG_WARN, "input event rejected: " + status.message);
        return status.ok();
    }

    xgc_status update_and_publish(double now_sec, uint64_t round) {
        runtime.update(now_sec);
        for (const auto& publication : runtime.publishedEstimates()) {
            const double stamp = publication.stamp_sec;
            const auto& out = publication.output;
            hover_thrust_native::xgc_hover_thrust_v1 msg{};
            msg.stamp = stamp;
            msg.hover_thrust = out.hover_thrust;
            msg.raw_hover_thrust = out.raw_hover_thrust;
            msg.initial_hover_thrust = out.initial_hover_thrust;
            msg.thrust_to_acceleration = out.thrust_to_acceleration;
            msg.last_estimate_stamp = out.last_estimate_stamp_sec;
            msg.state = out.state;
            msg.flags = out.flags;
            msg.sample_used = out.sample_used ? 1u : 0u;
            if (host->publish(host->host, kHoverThrust, round,
                              reinterpret_cast<const uint8_t*>(&msg), sizeof msg) != XGC_OK) {
                return XGC_ERR;
            }
        }
        return XGC_OK;
    }

    xgc_status step(const xgc_step_ctx* ctx) {
        for (auto& queue : pending)
            queue.clear();
        xgc_sample_view view;
        for (uint32_t port : {kImu, kAttitudeTarget, kPose, kAttitudeTargetFull}) {
            while (host->next(host->host, port, &view) == XGC_OK) {
                if (port == kImu && view.len == sizeof(xgc_imu_v1)) {
                    xgc_imu_v1 m;
                    std::memcpy(&m, view.data, sizeof m);
                    pending[port].push_back({m.stamp, port, m.accel[2], false});
                } else if (port == kAttitudeTarget && view.len == sizeof(xgc_attitude_target_v1)) {
                    xgc_attitude_target_v1 m;
                    std::memcpy(&m, view.data, sizeof m);
                    pending[port].push_back({m.stamp, port, m.thrust, m.ignore_thrust != 0});
                } else if (port == kAttitudeTargetFull &&
                           view.len == sizeof(xgc_attitude_target_v2)) {
                    xgc_attitude_target_v2 m;
                    std::memcpy(&m, view.data, sizeof m);
                    // Only this transport conversion is new. The estimator receives the
                    // same normalized command and IGNORE_THRUST semantics as /1.
                    pending[port].push_back(
                        {m.stamp, kAttitudeTarget, m.thrust, (m.type_mask & 64) != 0});
                } else if (port == kPose && view.len == sizeof(xgc_pose_v1)) {
                    xgc_pose_v1 m;
                    std::memcpy(&m, view.data, sizeof m);
                    pending[port].push_back({m.stamp, port, m.position[2], false});
                } else {
                    log(XGC_LOG_WARN, "dropped a sample with the wrong payload size");
                }
            }
        }
        // Merge FIFO HEADS only. Sorting all samples launders a source-port
        // clock regression (10.2 -> 10.1) into a healthy 10.1 -> 10.2 stream.
        // Every port's delivery order remains intact, including nonfinite and
        // backward source stamps; shared runtime ingestion classifies them.
        const auto next_input = [&](Pending& sample) {
            std::deque<Pending>* chosen = nullptr;
            for (auto& queue : pending) {
                if (queue.empty())
                    continue;
                if (!chosen || (std::isfinite(queue.front().stamp) &&
                                (!std::isfinite(chosen->front().stamp) ||
                                 queue.front().stamp < chosen->front().stamp)))
                    chosen = &queue;
            }
            if (!chosen)
                return false;
            sample = chosen->front();
            chosen->pop_front();
            return true;
        };
        Pending sample{};
        if (input_time) {
            // Deterministic replay: the runtime clock is the stamp of each sample.
            while (next_input(sample)) {
                apply(sample);
                if (update_and_publish(sample.stamp, ctx->round) != XGC_OK)
                    return XGC_ERR;
            }
            return XGC_OK;
        }
        while (next_input(sample))
            apply(sample);
        return update_and_publish(static_cast<double>(ctx->now) * 1e-9, ctx->round);
    }
};

// No exception may cross the C ABI. A caught one is logged through the host
// (so health.jsonl says why) and becomes XGC_ERR, which faults the plugin.
template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
    try {
        return f();
    } catch (const std::exception& e) {
        host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": " + e.what()).c_str());
        return XGC_ERR;
    } catch (...) {
        host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": unknown exception").c_str());
        return XGC_ERR;
    }
}

void* create(const xgc_host_api* host) {
    try {
        auto* self = new EstHoverThrust{host, {}, false, {}};
        return self;
    } catch (...) {
        return nullptr;
    }
}

xgc_status configure(void* p, const char* config) {
    auto* self = static_cast<EstHoverThrust*>(p);
    return guarded(self->host, "configure", [&] {
        const std::string text = config ? config : "";
        hte::HoverThrustEstimatorConfig c;
        bool ok = xgc_rt_config::number(text, "gravity", &c.gravity) &&
                  xgc_rt_config::number(text, "initial_hover_thrust", &c.initial_hover_thrust) &&
                  xgc_rt_config::number(text, "rho2", &c.rho2) &&
                  xgc_rt_config::number(text, "min_hover_thrust", &c.min_hover_thrust) &&
                  xgc_rt_config::number(text, "max_hover_thrust", &c.max_hover_thrust) &&
                  xgc_rt_config::number(text, "min_altitude", &c.min_altitude) &&
                  xgc_rt_config::number(text, "sample_timeout", &c.sample_timeout) &&
                  xgc_rt_config::boolean(text, "filter_enabled", &c.filter_enabled) &&
                  xgc_rt_config::number(text, "filter_cutoff_hz", &c.filter_cutoff_hz) &&
                  xgc_rt_config::number(text, "input_rate_low_hz", &c.input_rate_low_hz) &&
                  xgc_rt_config::number(text, "publish_rate_hz", &c.publish_rate_hz) &&
                  xgc_rt_config::number(text, "raw_update_rate_hz", &c.raw_update_rate_hz);
        std::string source = "\"session\"";
        xgc_rt_config::value(text, "time_source", &source);
        if (source != "\"session\"" && source != "\"input\"")
            ok = false;
        if (!ok) {
            self->log(XGC_LOG_ERROR, "invalid est-hover-thrust config");
            return XGC_ERR;
        }
        self->input_time = source == "\"input\"";
        self->runtime.setConfig(c);
        return XGC_OK;
    });
}

xgc_status activate(void*) {
    return XGC_OK;
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
    auto* self = static_cast<EstHoverThrust*>(p);
    return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) {
    return XGC_OK;
}

void destroy(void* p) {
    delete static_cast<EstHoverThrust*>(p);
}

const char* domain_state(void* p) {
    switch (static_cast<EstHoverThrust*>(p)->runtime.snapshotOutput().state) {
        case hte::state_type::SelfCheck:
            return "self_check";
        case hte::state_type::Ground:
            return "ground";
        case hte::state_type::Airborne:
            return "airborne";
        default:
            return "unknown";
    }
}

const xgc_port_decl kPorts[] = {
    {"imu", XGC_PORT_IN, "xgc.imu/1", XGC_QOS_STATE},
    {"attitude_target", XGC_PORT_IN_OPTIONAL, "xgc.attitude_target/1", XGC_QOS_STATE},
    {"pose", XGC_PORT_IN, "xgc.pose/1", XGC_QOS_STATE},
    {"hover_thrust", XGC_PORT_OUT, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"attitude_target_full", XGC_PORT_IN_OPTIONAL, "xgc.attitude_target/2", XGC_QOS_STATE},
};

const xgc_plugin_vtbl kVtbl = {create,     configure, activate,    step,
                               deactivate, destroy,   domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, 5u, "est-hover-thrust", "0.2.0", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(
    void) {
    return &kDescriptor;
}
