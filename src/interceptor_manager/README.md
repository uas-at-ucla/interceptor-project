# interceptor_manager

`manager_node` takes relative position setpoints on `/setpoint` and flies the
drone there through MAVROS (PX4 offboard mode).

## Run

With MAVROS connected to PX4 (e.g. `ros2 launch interceptor_bringup mavros.launch.py`):

```bash
ros2 run interceptor_manager manager_node
```

## Debugging from the terminal

Each message is an offset in metres from where the drone is **now**:

```bash
# Take off / climb 3 m
ros2 topic pub --once /setpoint geometry_msgs/msg/Point "{x: 0.0, y: 0.0, z: 3.0}"

# Move 5 m east (local frame) or 5 m forward (body frame)
ros2 topic pub --once /setpoint geometry_msgs/msg/Point "{x: 5.0, y: 0.0, z: 0.0}"

# Watch what is sent to PX4
ros2 topic echo /mavros/setpoint_position/local
```

Always pass `--once`. Without it, `ros2 topic pub` repeats the message at 1 Hz
and each repeat applies the offset again, so the target keeps moving.

## Behaviour

- Once a local pose is available, the node streams a position target to
  `/mavros/setpoint_position/local`. With no active command the target is the
  current position, so switching into OFFBOARD never causes a jump.
- When a `/setpoint` arrives, the target becomes current position + offset,
  keeping the current yaw. With `auto_engage` set, the node then switches to
  OFFBOARD and arms, retrying once per second.
- If the vehicle leaves OFFBOARD (pilot or failsafe) or disarms, the active
  command is dropped and the node stops trying to engage.

## Parameters

| Name               | Default   | Meaning |
|--------------------|-----------|---------|
| `setpoint_frame`   | `local`   | `local`: ENU (x east, y north, z up). `body`: x forward, y left, z up, rotated by current yaw. |
| `auto_engage`      | `true`    | Switch to OFFBOARD and arm when a setpoint arrives. **Set `false` on real hardware** and engage from the RC/GCS instead. |
| `max_offset_m`     | `50.0`    | Reject offsets longer than this. |
| `setpoint_rate_hz` | `20.0`    | Rate of the setpoint stream to MAVROS (PX4 needs > 2 Hz). |
| `mavros_ns`        | `/mavros` | MAVROS namespace. |

Example: `ros2 run interceptor_manager manager_node --ros-args -p setpoint_frame:=body -p auto_engage:=false`

## Code layout

- `include/interceptor_manager/mavros_interface.hpp`, `src/mavros_interface.cpp`:
  all MAVROS topics and services. New command types (velocity, attitude, etc.)
  go here.
- `src/manager_node.cpp`: the decision logic (setpoint handling, the
  hold/engage state). Other inputs such as tracker output plug in here.
