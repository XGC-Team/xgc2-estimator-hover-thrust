#include <hover_thrust_estimator/native/hover_thrust_wire.h>
#include <type_traits>

static_assert(std::is_standard_layout<hover_thrust_native::xgc_hover_thrust_v1>::value,
              "wire must remain standard layout");
static_assert(std::is_trivially_copyable<hover_thrust_native::xgc_hover_thrust_v1>::value,
              "wire must remain byte-copyable");
int main() {
    hover_thrust_native::xgc_hover_thrust_v1 output{};
    return sizeof output == 64 && output.reserved == 0 ? 0 : 1;
}
