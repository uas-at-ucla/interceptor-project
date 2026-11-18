# Manager Node — Design Decisions (Phases 1 & 2)

This is a short record of how the manager node works and why it was built this way. The code should be readable on its own; this explains the reasoning behind it.

## What the manager does

The manager is the drone's decision-maker. It reads drone state from MAVROS and target estimates from the Tracker, decides what the drone should do, and sends position setpoints to MAVROS. It runs one state machine on a fixed 20 Hz timer.

```
WAIT_FOR_READY ──(connected + armed + OFFBOARD)──> TAKEOFF ──(at altitude)──> SWEEP
                                                                 │   ▲
                                                  target seen    │   │  5 s with no target
                                                                 ▼   │
                                              FOLLOW ──(lost)──> SEARCH
                                                 ▲                  │
                                                 └──(target seen)───┘

INTERCEPT: exists for Phase 3, nothing enters it yet.
```

| Stage on our diagram | State(s) in code |
|---|---|
| Patrol | `TAKEOFF`, `SWEEP`, `SEARCH` |
| Follow + Lock | `FOLLOW` |
| Intercept | `INTERCEPT` (placeholder) |

If MAVROS loses its link to PX4, or the drone is disarmed or leaves OFFBOARD (for example, the safety pilot takes over), the manager goes back to `WAIT_FOR_READY` and keeps sending "hold where you are" setpoints. Then re-entering OFFBOARD never pulls the drone toward an old target. The sweep remembers its place.

## Decisions and why

**Setpoints are absolute positions in MAVROS's local frame.** The local frame is meters east, north, and up from where PX4's position estimator started, roughly the takeoff point. PX4 handles this frame most reliably. The Tracker already gives absolute positions, so no conversion is needed, and an absolute point doesn't go stale while the drone moves.

**One setpoint topic for every state.** We use `/mavros/setpoint_raw/local`. Sweeping sends a position. Following sends position plus the target's velocity, so the drone keeps pace instead of always lagging behind. Speed limits are left to PX4's own parameters, so the manager doesn't duplicate them.

**The manager never predicts.** All estimation lives in the Tracker. The Tracker publishes at a fixed rate and fills gaps between detections with its own prediction.

**The manager decides when the target is lost.** It owns the state machine, so it owns that call. The Tracker only reports facts, including when it last *actually* saw the target (`last_seen`). If `last_seen` is older than 1 s, the target counts as lost. The same check also catches a crashed Tracker, because `last_seen` stops updating.

**Following holds a fixed standoff distance at the target's altitude.** The target is in the air. The drone stays a fixed distance from the target, on whatever side it is already on, and faces the target so the camera keeps it in view. A minimum altitude stops it from following a target into the ground. While that floor is holding the drone above the target, the vertical part of the target's velocity is not fed forward, because it would fight the fixed altitude.

**Lost target: spin, then go back to sweeping.** The drone holds its position, which is already near where the target was last seen, and turns one full rotation over 5 s. If the target reappears, it goes back to `FOLLOW`. Otherwise it goes back to `SWEEP`.

**The sweep fits the circle and resumes where it left off.** The snake pattern's lanes are clipped to the circular search area. After a chase, the drone flies straight to the sweep waypoint it was heading to. When the sweep finishes, it starts over.

**Arming and OFFBOARD are outside the manager.** The manager never arms the drone or changes its mode. It waits until the drone is armed and in OFFBOARD. In sim, `interceptor_bringup/scripts/arm_offboard.py` does both: it waits for MAVROS and for the manager's setpoints (PX4 rejects OFFBOARD without them), then requests OFFBOARD and arms. It is a sim/test tool only. On real flights the safety pilot arms and switches to OFFBOARD from the RC.

**Camera and command services are stubs for now.** The clients exist but nothing calls them. `/camera_toggle` uses the standard on/off service type instead of a "toggle", because a toggle can get out of sync with the camera's real state. `/camera_pos` still needs its service type defined.

**All tunable numbers live in one YAML file.** These are the sweep center, radius, altitude, lane spacing, standoff distance, timeouts, and so on. The sweep center defaults to (0, 0), which is the drone's start point.

**We verify in Isaac Sim** with PX4 SITL and MAVROS. All nodes use wall time (`use_sim_time: false`), so timestamps are comparable: Isaac doesn't publish `/clock`, and MAVROS stamps its messages in wall time. To simulate a pilot takeover in SITL, switch to `AUTO.LOITER`. PX4 rejects `POSCTL` there because there is no RC or joystick input.

## Contract for the Tracker: `/target_data`

The manager depends on these rules, so the Tracker must follow them:

| Field | Meaning |
|---|---|
| `header.stamp` | Time of this estimate |
| `header.frame_id` | Same frame as `/mavros/local_position/pose` |
| `position` | Target position, meters, local ENU |
| `velocity` | Target velocity, m/s, local ENU |
| `last_seen` | Time of the last real camera detection used by the filter |

- Publish only once a track exists, then at a fixed rate, including predicted updates between detections.
- Update `last_seen` only when a real detection arrives, never for predictions.
- Stamp `header.stamp` and `last_seen` in wall time (`use_sim_time: false`), like every other node.
- Convert camera-relative detections into the local frame inside the Tracker, using the drone's pose.

## Open items and ideas

- **Lane spacing for an aerial target.** The camera faces forward, not down. So spacing comes from how far away the camera can reliably detect the target ($R_d$), not from a ground footprint. A straight pass covers a strip about $2R_d\sin(\theta/2)$ wide, where $\theta$ is the camera's horizontal field of view. Example: $\theta = 90°$ and $R_d = 50$ m gives about 71 m. **Measure $R_d$ in Isaac Sim** before tuning.
- **Sweep time is fine at 500 ft (152 m) radius.** With 71 m lanes, the path is about $\pi \cdot 152^2 / 71 \approx 1$ km, around 2 minutes at 10 m/s. The 500 m radius would be the problem.
- **Faster search ideas to try later:**
  - **Spin at the center first.** One 360° turn over the center covers the whole area if $R_d$ is close to the radius. It takes seconds.
  - **Concentric circles instead of a snake.** The drone slows down at every sharp turn, and the snake has two sharp turns per lane. Orbits have none.
  - **Start where the target is likely to be,** if we know where it launches or which way it tends to fly.
- **`/camera_pos` service type** still needs to be defined.
