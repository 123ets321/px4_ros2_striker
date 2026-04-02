# PX4 ROS 2 Fixed-Wing APN Striker

This package provides a high-performance **Augmented True Proportional Navigation (TPN) Strike** node for Fixed-Wing UAVs running PX4 via ROS 2.

It receives global target coordinates (`Lat, Lon, Alt`) from a Ground Control Station (GCS) and dynamically commands split-channel `VehicleAttitudeSetpoints` directly to the PX4 flight controller to execute extremely aggressive, missile-like interception pathways. It also includes full support for strike aborts and dynamic waypoint recovery.

---

## 🚀 Theory of Operation

### 1. True Proportional Navigation (TPN)
Traditional Pure Proportional Navigation (PPN) generates commands perpendicular to the Line-Of-Sight (LOS) vector. For aircraft approaching off-axis, this results in purely longitudinal force vectors which fail to trigger roll/bank commands.

This package uses **True Proportional Navigation** which crosses the LOS rotation rate ($\Omega$) with the aircraft's own velocity vector ($\hat{v}$):
$$ a_{PN} = N \cdot |V_{drone}| \cdot (\Omega \times \hat{v}) $$
This mathematically guarantees the commanded acceleration is always exactly 90° lateral to the airframe, throwing the aircraft into an immediate maximum-bank maneuver to re-acquire off-axis targets.

### 2. Attitude Inversion
Unlike multicopters which can accept arbitrary velocity setpoints, fixed-wing aircraft must bank to turn. The node projects the 3D-NED $a_{cmd}$ navigation vector onto the aircraft's body axes, calculating continuous roll, pitch, and yaw targets directly for the PX4 attitude controllers while strictly clamping them to structural limits (e.g., $MAX\_ROLL = 55^{\circ}$).

### 3. Terminal Phase "Open Loop"
As the physical target completely fills the sensor/navigation frame during the final milliseconds of flight, noise dramatically increases. The striker relies on an exact Time-to-Go ($t_{go}$) prediction. At $t_{go} < 150\text{ms}$, it enters **TERMINAL** mode. The ROS 2 node permanently halts, capitalizing on the PX4 `COM_OF_LOSS_T` watchdog sequence. The drone harmlessly locks its control surfaces on the last optimal trajectory and impacts kinetically prior to the failsafe event.

---

## ⚙️ State Management Workflow

The node is completely robust and re-triggerable over the lifecycle of the entire flight.

- **`IDLE`**: The node primes the OFFBOARD state while waiting for a mission. 
- **`STRIKE`**: Computes TPN in real-time. Drives PX4 via Offboard `VehicleAttitudeSetpoint` commands.
- **`TERMINAL`**: (Absorbing State). The node goes silent exactly 150ms before kinetic impact.
- **`RECOVERY`**: Instantly drops the drone out of an active dive, reverting to standard `TrajectorySetpoint` position control to fly to the safe-point.
- **`RECOVERED`**: Waypoint reached (default `< 10m`). Automatically commands PX4 into `AUTO.LOITER` or `AUTO.RTL` securely handing flight authority back to the autopilot. Node waits patiently to be re-triggered into STRIKE.

---

## 📡 Topics & Messages

### Required Action Types (Custom)
*   **Strike Action:** `px4_ros2_striker/action/Strike`
*   **Recover Action:** `px4_ros2_striker/action/Recover`

### Action Servers
*   `/strike_action` — Triggers a kinetic dive on a `Lat/Lon/Alt` coordinate. Returns boolean `hit` and `miss_distance_m`. Provides high-speed 50Hz telemetry feedback.
*   `/recover_action` — Triggers an abort and flies geometrically to a `Lat/Lon/Alt` safety coordinate. Provides distance-to-waypoint feedback.

### Subscribers 
*   `/fmu/out/vehicle_local_position` — Odometry telemetry.
*   `/fmu/out/vehicle_status` — State monitoring.
*   `/fmu/out/vehicle_attitude`
*   `/fmu/out/airspeed_validated`

### Publishers
*   `/fmu/in/vehicle_command` — State machine control mode requests.
*   `/fmu/in/offboard_control_mode` — OFFBOARD heartbeat constraint matrix.
*   `/fmu/in/vehicle_attitude_setpoint` — Used during **STRIKE** mode.
*   `/fmu/in/trajectory_setpoint` — Used during **RECOVERY** mode.
*   `/drone_path` — (`nav_msgs/msg/Path`) Published by the attached visualization node for RViz2.

---

## 💻 Installation & Usage

### 1. Requirements
*   **ROS 2 Humble / Iron / Jazzy**
*   **PX4 Autopilot (v1.14+)**

### 2. Build Instructions
```bash
cd ~/ETS/ws_shadow_m
colcon build --packages-select px4_ros2_striker
source install/setup.bash
```

### 3. Execution
Launch the entire striking suite (including the 3D trajectory visualization):
```bash
ros2 launch px4_ros2_striker striker_rviz.launch.py
```
*(Optionally, you can run just the bare action server: `ros2 run px4_ros2_striker striker_action_server`)*

### 4. Sending Commands (GCS Simulation)

To trigger a kinetic strike dive (and stream feedback telemetry):
```bash
ros2 action send_goal --feedback /strike_action px4_ros2_striker/action/Strike "{latitude: 47.398, longitude: 8.545, altitude: 0.0}"
```

To abort a dive and command geometric recovery (at 50m altitude with a 10m arrival radius):
```bash
```bash
ros2 action send_goal --feedback /recover_action px4_ros2_striker/action/Recover "{latitude: 47.397, longitude: 8.546, altitude: 50.0, final_mode: 'hold', arrival_radius_m: 10.0}"
```

---

## 🛑 Flight Constraints & Safety Margins

### 1. Minimum Flight Conditions for a Successful Strike

These are the conditions the vehicle **must satisfy** when the strike goal is sent:

#### Altitude
```text
Minimum:  target_altitude + 30m
Ideal:    target_altitude + 80–150m
```
The pitch controller needs room to establish a stable dive angle before the terminal phase. At 30m clearance you have roughly 2–3 seconds of guidance time at 15 m/s. Below this the guidance has no time to converge before the proximity fuse triggers on a non-optimal approach angle.

#### Airspeed
```text
Minimum:  1.3 × stall speed  (typically 18–22 m/s for a 2kg FW)
Ideal:    25–40 m/s
```
Below 1.3Vs the roll authority drops, banked turns lose lift, and the pitch tracker commands nose-down into an accelerated stall. The `AIRSPEED_FALLBACK = 15.0` in the code is dangerously close to stall for most fixed-wing platforms — verify this against your actual Vs.

#### Range at Goal Acceptance
```text
Minimum:  150m  (at 25 m/s → ~6s of guidance time)
Ideal:    300–1000m
```
Below 150m there is insufficient time for the PN law to rotate the velocity vector onto a collision course. The drone will likely fly past the target and require a second pass.

#### Heading Geometry
```text
Avoid:   heading_error > 120° at goal acceptance
         (tail-chase condition — recovery maneuver wastes 3–5 seconds)
Ideal:   heading_error < 60° relative to LOS
```
The tail-chase recovery in the code pulls the drone laterally toward the LOS, but at high speed (50+ m/s) a 180° turn has a radius of ~300m — the drone may exit the engagement envelope before re-acquiring.

#### LOS Elevation to Target
```text
Minimum depression angle:  5°  (target must be measurably below drone)
Maximum depression angle:  35° (steeper than this exceeds MAX_PITCH_RAD and guidance saturates)
Ideal range:               10°–25°
```
At 0° depression (target at same altitude) the pitch channel commands nothing — `los_el ≈ 0`, `fpa ≈ 0`, `pitch_cmd ≈ 0`. The drone flies level and misses entirely.

#### Summary Table — Minimum Conditions

| Parameter | Minimum | Ideal |
|---|---|---|
| Altitude above target | 30 m | 80–150 m |
| Airspeed | 1.3 × Vs | 25–40 m/s |
| Range at goal send | 150 m | 300–1000 m |
| Heading error to LOS | < 120° | < 60° |
| LOS depression angle | 5° | 10°–25° |
| Nav state | OFFBOARD confirmed | — |
| GPS fix | 3D fix, HDOP < 2 | RTK |

---

### 2. Maximum Conditions for a Safe Abort

"Safe abort" means sending a `CancelGoal` or a `/recover_action` goal and having the vehicle recover controllably. These are the limits beyond which abort is either impossible or results in terrain impact.

#### Hard Abort Boundary — No Recovery Possible
```text
R < HIT_RADIUS (5m)     → TERMINAL already entered, abort rejected
t_go < 0.15s            → TERMINAL already entered, abort rejected
Altitude < 20m AGL      → even if abort accepted, insufficient altitude
                           for pull-up and level flight recovery
```

#### Safe Abort Envelope

**Range:**
```text
Minimum safe abort range:  80m
```
At 80m and 25 m/s you have ~3.2 seconds. The recovery position setpoint takes effect within 1 OFFBOARD prime cycle (~1s). The remaining 2.2s is enough for the FW position controller to begin a pull-up. Below 80m the drone will hit the ground before PX4's position controller can arrest the dive.

**Dive angle (FPA) at abort:**
```text
Maximum safe FPA at abort:  −20°
```
At −20° and 25 m/s the aircraft descends at ~8.5 m/s. PX4's position controller can command a pull-up of roughly 4–5 m/s² → arrest takes ~2 seconds → altitude loss ~8.5m. If you have less than 15m AGL clearance at abort, impact is likely regardless of the command.

**Airspeed at abort:**
```text
Maximum safe airspeed:  1.5 × cruise (typically 45–55 m/s for most platforms)
```
Above this, the control surfaces may saturate trying to pull up. PX4's attitude controller will clip the pitch rate and the pull-up takes longer — increasing the altitude loss window.

**Bank angle at abort:**
```text
Maximum safe bank angle:  45°
```
In a banked turn, lift is reduced by `cos(bank)`. At 45° bank you have 0.7g vertical lift — barely enough to maintain level flight. If abort is sent while banked >45° the position controller must first roll wings-level before pulling up, adding 1–2 seconds of altitude loss.

#### Summary Table — Maximum Safe Abort Conditions

| Parameter | Maximum for Safe Abort | Beyond This |
|---|---|---|
| Range | > 80m | Impact before recovery |
| Altitude AGL | > 20m | No room for pull-up |
| FPA (dive angle) | > −20° | Pull-up altitude loss exceeds clearance |
| Airspeed | < 1.5 × cruise | Control surface saturation |
| Bank angle | < 45° | Roll-out delay causes altitude loss |
| Time-to-go | > 0.5s (recommended) | TERMINAL may already be entered |

The **recommended abort trigger in a mission manager** is therefore:
```cpp
if (feedback.range_m > 80.0 &&
    feedback.fpa_deg > -20.0 &&
    altitude_agl > 20.0 &&
    feedback.t_go_s > 0.5) {
    strike_client->async_cancel_goal(goal_handle);
}
```
