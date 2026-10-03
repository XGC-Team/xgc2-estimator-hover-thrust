#ifndef HOVER_THRUST_NATIVE_WIRE_H
#define HOVER_THRUST_NATIVE_WIRE_H

#include <stddef.h>
#include <stdint.h>

/* Domain-owned "xgc.hover_thrust/1" wire. Exact 64-byte field/layout move from
 * xgc_schemas_v1.h; no Runtime/ROS/library or estimator policy dependency.
 * C++ scopes this name to the owner so transitional input-only legacy headers
 * can coexist without supplying the estimator's output declaration. In C the
 * record is named xgc_hover_thrust_v1. The serialized ABI is identical. */
#ifdef __cplusplus
namespace hover_thrust_native {
#endif

typedef struct xgc_hover_thrust_v1 {
    double stamp;
    double hover_thrust;
    double raw_hover_thrust;
    double initial_hover_thrust;
    double thrust_to_acceleration;
    double last_estimate_stamp;
    uint32_t state;
    uint32_t flags;
    uint32_t sample_used;
    uint32_t reserved;
} xgc_hover_thrust_v1;

#ifdef __cplusplus
#define HTE_WIRE_ASSERT(condition, message) static_assert(condition, message)
#else
#define HTE_WIRE_ASSERT(condition, message) _Static_assert(condition, message)
#endif
HTE_WIRE_ASSERT(sizeof(xgc_hover_thrust_v1) == 64, "xgc.hover_thrust/1 size");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, stamp) == 0, "stamp offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, hover_thrust) == 8, "hover offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, raw_hover_thrust) == 16, "raw offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, initial_hover_thrust) == 24, "initial offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, thrust_to_acceleration) == 32, "ratio offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, last_estimate_stamp) == 40, "estimate stamp offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, state) == 48, "state offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, flags) == 52, "flags offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, sample_used) == 56, "sample-used offset");
HTE_WIRE_ASSERT(offsetof(xgc_hover_thrust_v1, reserved) == 60, "reserved offset");
#undef HTE_WIRE_ASSERT

#ifdef __cplusplus
}  // namespace hover_thrust_native
#endif
#endif
