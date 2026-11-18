# interceptor_manager

`manager_node` is the drone's decision-maker. It runs one state machine at
20 Hz and sends absolute position setpoints (local ENU) to MAVROS on
`/mavros/setpoint_raw/local`. The design reasoning is in
`manager_node_decisions.md` at the repository root.

The manager never arms the drone or changes its flight mode. It waits until
the drone is armed and in OFFBOARD.

## Run

1. Start the sim (see `worlds/README.md`).
2. Start MAVROS.
3. `ros2 launch interceptor_manager manager.launch.py`

All tunable numbers are in `config/manager.yaml`.

`use_sim_time` defaults to `false`. Isaac Sim doesn't publish `/clock`, and
MAVROS stamps messages in wall time, so every node uses wall time. Any node
that publishes `/target_data` must do the same, or `last_seen` won't be
comparable.

## Interfaces

| Direction | Topic / Service | Type |
|---|---|---|
| Sub | `/mavros/state` | `mavros_msgs/State` |
| Sub | `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` (sensor-data QoS) |
| Sub | `/target_data` | `interceptor_msgs/TargetData` |
| Pub | `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` |
| Client (stub) | `/camera_toggle` | `std_srvs/SetBool` |

## Current behaviour

- Publishes nothing until the first pose arrives from MAVROS.
- `WAIT_FOR_READY`: holds the current position and yaw. Moves to `TAKEOFF`
  once MAVROS is connected and the drone is armed and in OFFBOARD.
- `TAKEOFF`: climbs straight up from where it entered the state to
  `sweep_altitude`, keeping its heading. Moves to `SWEEP` once within
  `waypoint_tolerance` of that altitude.
- `SWEEP`: flies a snake pattern clipped to the search circle (lanes along x,
  `lane_spacing` apart), facing the direction of travel. Repeats from the
  start after the last waypoint. The current waypoint is kept across a pilot
  takeover, so the sweep resumes where it left off.
- `FOLLOW`, `SEARCH`, `INTERCEPT` currently hold the current position; later
  steps fill them in.
- Losing armed or OFFBOARD from any state returns to `WAIT_FOR_READY`.
