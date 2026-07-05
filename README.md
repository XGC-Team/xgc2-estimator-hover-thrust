# Hover Thrust Estimator

ROS1 package for estimating the normalized hover thrust used by PX4/MAVROS
multirotor controllers.

The estimator subscribes to MAVROS IMU, target attitude thrust, and local pose
topics. It publishes a latched `hover_thrust_estimator/HoverThrustEstimate`
message containing the current estimator state, health flags, and filtered
hover thrust estimate.

## State Model

The runtime state machine has two regions:

- `HEALTH`: one internal state, `HealthMonitor`, that continuously classifies
  the latest inputs.
- `ESTIMATION`: four externally visible estimator states.

Only the four estimation states are exported in `HoverThrustEstimate.state`.

```mermaid
flowchart LR
    subgraph HEALTH["HEALTH region"]
        HM["HealthMonitor<br/>classify latest input snapshot"]
    end

    subgraph ESTIMATION["ESTIMATION region<br/>public HoverThrustEstimate.state"]
        SC["SelfCheck<br/>state = 0<br/>inputs not ready<br/>publish held output"]
        G["Ground<br/>state = 1<br/>below min_altitude<br/>freeze target"]
        A["Airborne<br/>state = 2<br/>healthy and airborne<br/>update RLS"]
        F["Fault<br/>state = 9<br/>state-machine fault<br/>publish fault flag"]
    end

    HM -->|"HEALTH_TO_SELF_CHECK"| SC
    HM -->|"HEALTH_TO_GROUND"| G
    HM -->|"HEALTH_TO_AIRBORNE"| A
    HM -->|"HEALTH_TO_FAULT"| F
```

The internal state ids are different from the message values:

| Internal state | Internal id |
| --- | ---: |
| `HealthMonitor` | `1` |
| `SelfCheck` | `10` |
| `Ground` | `11` |
| `Airborne` | `12` |
| `Fault` | `19` |

## Health Classification

`HealthMonitor` maps the latest input snapshot to one of the four estimation
states. The first failing gate wins.

```mermaid
flowchart TD
    Start["Latest input snapshot"] --> FaultReq{"fault_requested?"}
    FaultReq -->|"yes"| Fault["Fault<br/>FLAG_STATE_MACHINE_FAULT"]
    FaultReq -->|"no"| Missing{"IMU, thrust,<br/>or altitude missing?"}

    Missing -->|"yes"| SelfMissing["SelfCheck<br/>missing input flags"]
    Missing -->|"no"| TimeJump{"timestamp jump<br/>or backward sample?"}

    TimeJump -->|"yes"| SelfTime["SelfCheck<br/>FLAG_TIME_JUMP"]
    TimeJump -->|"no"| Stale{"sample age exceeds<br/>sample_timeout?"}

    Stale -->|"yes"| SelfStale["SelfCheck<br/>stale input flags"]
    Stale -->|"no"| LowRate{"input rate below<br/>input_rate_low_hz?"}

    LowRate -->|"yes"| SelfRate["SelfCheck<br/>FLAG_INPUT_RATE_LOW"]
    LowRate -->|"no"| Invalid{"non-finite input,<br/>ignored thrust,<br/>or thrust out of range?"}

    Invalid -->|"yes"| SelfInvalid["SelfCheck<br/>invalid input flags"]
    Invalid -->|"no"| LowAlt{"altitude < min_altitude?"}

    LowAlt -->|"yes"| Ground["Ground<br/>FLAG_BELOW_MIN_ALTITUDE<br/>FLAG_GROUND_HOLD"]
    LowAlt -->|"no"| Airborne["Airborne<br/>ready for RLS update"]
```

## Transition Rules

The estimation region follows the health target whenever the target differs
from the current state. All non-self transitions are explicit:

```mermaid
stateDiagram-v2
    [*] --> SelfCheck

    SelfCheck --> Ground: HEALTH_TO_GROUND
    SelfCheck --> Airborne: HEALTH_TO_AIRBORNE
    SelfCheck --> Fault: HEALTH_TO_FAULT

    Ground --> SelfCheck: HEALTH_TO_SELF_CHECK
    Ground --> Airborne: HEALTH_TO_AIRBORNE
    Ground --> Fault: HEALTH_TO_FAULT

    Airborne --> SelfCheck: HEALTH_TO_SELF_CHECK
    Airborne --> Ground: HEALTH_TO_GROUND
    Airborne --> Fault: HEALTH_TO_FAULT

    Fault --> SelfCheck: HEALTH_TO_SELF_CHECK
    Fault --> Ground: HEALTH_TO_GROUND
    Fault --> Airborne: HEALTH_TO_AIRBORNE

    note right of SelfCheck
      Inputs are not safe for estimation.
      RLS is not updated.
    end note

    note right of Ground
      Inputs are healthy, but altitude
      is below min_altitude.
      RLS is not updated.
    end note

    note right of Airborne
      The only state that updates
      the raw RLS estimate.
    end note
```

There are no explicit self-transitions. If the health target is already the
active estimation state, `HealthMonitor` does not post a transition event.

## State Actions

Each estimation state owns a small action path. Only `Airborne` can update the
raw RLS estimate; all states can publish an output snapshot when the publish
gate is due.

```mermaid
flowchart LR
    SC["SelfCheck"] --> SCEntry["entry:<br/>target = initial_hover_thrust<br/>reset publish gate"]
    SCEntry --> SCPub{"publish gate due?"}
    SCPub -->|"yes"| PubSC["publish held / filtered output"]
    SCPub -->|"no"| HoldSC["hold output"]

    G["Ground"] --> GEntry["entry:<br/>freeze current target<br/>reset publish gate"]
    GEntry --> GPub{"publish gate due?"}
    GPub -->|"yes"| PubG["publish ground-hold output"]
    GPub -->|"no"| HoldG["hold output"]

    A["Airborne"] --> AEntry["entry:<br/>reset raw-update gate<br/>reset publish gate"]
    AEntry --> RawGate{"raw-update gate due<br/>and health ready?"}
    RawGate -->|"yes"| RLS["RLS update:<br/>acc_z + normalized_thrust"]
    RLS --> Target["raw estimate<br/>becomes output target"]
    RawGate -->|"no"| SkipRLS["skip RLS update"]
    Target --> APub{"publish gate due?"}
    SkipRLS --> APub
    APub -->|"yes"| PubA["drive output filter<br/>and publish estimate"]
    APub -->|"no"| HoldA["hold output"]

    F["Fault"] --> FEntry["entry:<br/>reset publish gate"]
    FEntry --> FPub{"publish gate due?"}
    FPub -->|"yes"| PubF["publish with<br/>FLAG_STATE_MACHINE_FAULT"]
    FPub -->|"no"| HoldF["hold output"]
```

## Estimation Update

The raw estimator fits a scalar relation:

```text
imu_acc_z ~= thrust_to_acceleration * normalized_thrust
hover_thrust = gravity / thrust_to_acceleration
```

Raw RLS updates are only attempted in `Airborne` and only at
`raw_update_rate`. The published hover thrust is driven toward the latest raw
target through the output low-pass model when publishing is due.

## Topics and Parameters

Default topics:

| Direction | Topic | Type |
| --- | --- | --- |
| Subscribe | `mavros/imu/data` | `sensor_msgs/Imu` |
| Subscribe | `mavros/setpoint_raw/target_attitude` | `mavros_msgs/AttitudeTarget` |
| Subscribe | `mavros/local_position/pose` | `geometry_msgs/PoseStamped` |
| Publish | `hover_thrust/estimate_state` | `hover_thrust_estimator/HoverThrustEstimate` |

Important timing and gating parameters:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `sample_timeout` | `0.2` | Maximum sample age before stale flags are set. |
| `input_rate_low_hz` | `5.0` | Minimum healthy input stream rate. |
| `min_altitude` | `0.5` | Altitude threshold below which the estimator enters `Ground`. |
| `loop_rate` | `1000.0` | Main runtime update loop rate. |
| `publish_rate` | `100.0` | Estimate-state publish rate. |
| `raw_update_rate` | `10.0` | RLS raw-estimate update rate. |
