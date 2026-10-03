#include <hover_thrust_estimator/native/hover_thrust_wire.h>
#include <string.h>

int main(void) {
    xgc_hover_thrust_v1 output = {0};
    unsigned char bytes[64] = {0};
    output.stamp = 123.5;
    output.hover_thrust = .3;
    output.raw_hover_thrust = .4;
    output.initial_hover_thrust = .3;
    output.thrust_to_acceleration = 32.68866666666667;
    output.last_estimate_stamp = 123.4;
    output.state = 12;
    output.flags = 8192;
    output.sample_used = 1;
    memcpy(bytes, &output, sizeof output);
    xgc_hover_thrust_v1 decoded;
    memcpy(&decoded, bytes, sizeof decoded);
    return decoded.state == 12 && decoded.flags == 8192 && decoded.sample_used == 1 &&
                   decoded.hover_thrust == .3 && decoded.reserved == 0 ? 0 : 1;
}
