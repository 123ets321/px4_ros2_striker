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

### Required Message Types
*   Strike & Recovery commands utilize standard: **`sensor_msgs/msg/NavSatFix`**

### Subscribers
*   `/strike_point` — Target coordinate.
*   `/recovery_point` — Abort/Safety coordinate. 
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
*(Optionally, you can run just the bare node: `ros2 run px4_ros2_striker striker_node`)*

### 4. Sending Commands (GCS Simulation)

To trigger a kinetic strike dive:
```bash
ros2 topic pub --once /strike_point sensor_msgs/msg/NavSatFix "{latitude: 47.398, longitude: 8.545, altitude: 0.0}"
```

To abort a dive and command geometric recovery (at 50m altitude):
```bash
ros2 topic pub --once /recovery_point sensor_msgs/msg/NavSatFix "{latitude: 47.397, longitude: 8.546, altitude: 50.0}"
```
