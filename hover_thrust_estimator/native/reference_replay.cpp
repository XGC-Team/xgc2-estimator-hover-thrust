// Reference for the equality test: feeds a recorded input sequence straight
// into HoverThrustEstimatorRuntime, the way the ROS HoverThrustInputProducer
// does per topic, with the runtime clock at each sample stamp, and prints
// every PUBLISH_ESTIMATE output the way HoverThrustOutputConsumer does
// (the shared runtime already drove and captured each publication). No ports, no
// host, no plugin code.
//
// stdin:  one sample per line: "<port> <stamp> <value> <ignore>" (shortest round-trip decimal)
//         port 0 imu accel z, 1 attitude-target thrust, 2 pose z
// stdout: one line per estimate:
//         "<stamp> <hover> <raw> <thr2acc> <last_estimate_stamp> <state> <flags> <sample_used>"
//         (%.17g, exact round trip)

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"

namespace hte = hover_thrust_estimator;

int main() {
    hte::HoverThrustEstimatorRuntime runtime;
    hte::HoverThrustEstimatorConfig config;  // defaults = the plugin's defaults
    runtime.setConfig(config);
    unsigned port = 0, ignore = 0;
    double stamp = 0, value = 0;
    while (std::scanf("%u %la %la %u", &port, &stamp, &value, &ignore) == 4) {
        const auto status = port == 0
                                ? runtime.ingestImu(value, stamp, stamp)
                                : port == 1 ? runtime.ingestThrust(value, ignore != 0, stamp, stamp)
                                            : runtime.ingestAltitude(value, stamp, stamp);
        if (!status.ok())
            return 1;
        runtime.update(stamp);
        for (const auto& publication : runtime.publishedEstimates()) {
            const double out_stamp = publication.stamp_sec;
            const auto& o = publication.output;
            std::printf("%.17g %.17g %.17g %.17g %.17g %u %u %u\n", out_stamp, o.hover_thrust,
                        o.raw_hover_thrust, o.thrust_to_acceleration, o.last_estimate_stamp_sec,
                        o.state, o.flags, o.sample_used ? 1u : 0u);
        }
    }
    return 0;
}
