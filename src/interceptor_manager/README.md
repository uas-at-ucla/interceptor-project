# interceptor_manager

`manager_node` takes relative position setpoints on `/setpoint` and flies the
drone there through MAVROS (PX4 offboard mode).

## Run

1. Start the sim.
2. `ros2 launch interceptor_bringup mavros.launch.py`
3. `ros2 run interceptor_manager manager_node`
4. Send a setpoint (an offset in metres from where the drone is now; x east, y north, z up):

```bash
ros2 topic pub --once /setpoint geometry_msgs/msg/Point '{x: 0.0, y: 0.0, z: 3.0}'
```

Always pass `--once`. Without it, `ros2 topic pub` repeats every second and
each repeat moves the target again.

## Behaviour

- Streams a position target to `/mavros/setpoint_position/local` at 20 Hz
  (the current position until the first setpoint arrives).
- After the first setpoint, switches to OFFBOARD and arms, retrying every second.

Not handled yet (add before real flights): it switches back to OFFBOARD if a
pilot switches out, re-arms after landing, and has no limit on offset size.
