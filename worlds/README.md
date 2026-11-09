# Pegasus worlds

Run these named worlds from the repository root using **Isaac Sim's Python**:

| World launcher | Behavior |
| --- | --- |
| `worlds/static_plate.py` | Grassy field, PX4 Iris, and a stationary marked paper plate. |
| `worlds/moving_plate.py` | Same field and Iris, with a plate following a smooth random path. |

The launchers define the simulations you run. Shared building code lives in
`worlds/_support/`: `scene.py` creates geometry and materials, `simulation.py`
starts Isaac/Pegasus/PX4, and `motion.py` generates the moving target's trajectory.

For ROS2 bridge support, prepare the simulator terminal using the
[ROS2 environment setup](#ros2-environment-setup) below before running these
commands. On this workstation, the existing `isaac_run` shell function does that
setup automatically, for example `isaac_run worlds/moving_plate.py --spawn-radius 0`.

## Static plate

```bash
~/isaacsim/python.sh worlds/static_plate.py
```

The scene has a flat grassy field (360 × 360 meters by default), daylight, and
Pegasus's stock **Iris** at `(0, 0, 0.07)` meters with its default dynamics and sensors. The target
is an upright **30 cm paper plate** with a **14 cm red square** taped to its front.
The plate has a raised rim, paper thickness, exposed tape ends, and a collider.

By default, the plate center is 2 meters above ground. Its horizontal position is
sampled uniformly by area within **500 feet (152.4 meters)** of the Iris spawn,
and its front faces spawn. Each launch prints its seed and initial coordinates
as `WORLD_METADATA`. Specify a seed to repeat placement:

```bash
~/isaacsim/python.sh worlds/static_plate.py --seed 42 --plate-height 3
```

Set the initial spawn radius with `--spawn-radius`, in **meters**. The default
is **152.4 meters (500 feet)**. For example, spawn within 10 meters of the Iris:

```bash
~/isaacsim/python.sh worlds/static_plate.py --spawn-radius 10
~/isaacsim/python.sh worlds/moving_plate.py --spawn-radius 10
```

The radius describes a disk around the Iris spawn, with positions sampled
uniformly by area. `--spawn-radius 0` places the plate directly above spawn.
Larger values than 152.4 are supported; the grass and ground collider grow to
cover the configured spawn and motion area.

For a close visual check, place the plate directly above spawn at `(0, 0, 2)`:

```bash
~/isaacsim/python.sh worlds/static_plate.py --spawn-radius 0 --plate-height 2
```

`--spawn-radius 0` also centers the initial viewport on the plate. Use a positive
radius or omit the option to restore random placement.

## Moving plate

```bash
~/isaacsim/python.sh worlds/moving_plate.py
```

The marked plate zigzags between nearby random 3D waypoints, easing to a stop and
turning at each waypoint. Each new heading differs by at least 60 degrees, with
short inward turns if needed at the field boundary. Direction changes happen
about every **0.9–2.1 seconds** by default, with random distances and upward/downward
movement. Defaults are a maximum translation speed of **6 m/s**, the same
**152.4 meter** horizontal roaming radius, and heights between **1.25 and 2.75
meters** (`--plate-height 2` plus/minus `--height-variation 0.75`). The initial
position uses `--spawn-radius`; subsequent motion stays within `--motion-radius`.
Gentle rocking changes the marker angle, while its front gradually turns toward
spawn. The paper, tape,
marker, and collision body move together.

For a repeatable demonstration close to the Iris:

```bash
~/isaacsim/python.sh worlds/moving_plate.py --spawn-radius 0 --seed 42 \
  --motion-radius 10 --speed 3 --turn-interval 1 --plate-height 3 --height-variation 1
```

Moving-world options:

| Input | Meaning |
| --- | --- |
| `--seed 42` | Repeat both the initial placement and random trajectory. |
| `--spawn-radius 5` | Choose the initial plate position within 5 meters of Iris spawn; use 0 to start directly above it. |
| `--motion-radius 10` | Keep all motion within 10 meters of Iris spawn. If spawn radius is omitted, initial placement also fits inside this radius. |
| `--speed 3` | Set the maximum translation speed to 3 m/s, including vertical movement. |
| `--turn-interval 1` | Change direction every 0.6–1.4 seconds; lower values give shorter, more frequent turns. |
| `--plate-height 3` | Set the starting height and middle of the altitude range, in meters. |
| `--height-variation 1` | Choose waypoint heights within ±1 meter of that height; use 0 for constant altitude. |

Spawn radius and motion radius are independent. For example, use
`--spawn-radius 5 --motion-radius 30` to start nearby and roam farther. An explicit
spawn radius must fit inside the motion radius. If motion radius is omitted, it
defaults to 152.4 meters, or the spawn radius when that is larger. If only a smaller
motion radius is supplied, the default spawn radius shrinks to match it.

Motion advances on physics steps, pauses with the simulation, and restarts from
the same seed when the timeline is stopped/reset. It is a scripted kinematic
target for vision experiments; it does not model disc lift, drag, or ballistic
flight. The current marker is the red square. Both worlds provide the moving or
stationary target only, with no vision tracking or automatic pursuit code.

## Viewing the target

Select `/World/Grassy/PaperPlate` in the Stage panel and press **F** to frame it.
For a close view, choose `/World/Grassy/PaperPlate/InspectionCamera` in the
viewport camera menu. This camera is mounted to the plate and moves with it;
use the normal viewport or a vehicle camera to observe target movement relative
to the observer. The initial free viewport is fixed after launch.

## PX4 and installation

**Every launch automatically starts PX4 SITL** and connects the Iris through
Pegasus's MAVLink backend as vehicle 0. The Iris starts disarmed; arm and command
it through your ground station or offboard program. The plate's motion is
independent of PX4. Use your existing Pegasus/isaac-uav installation, with PX4
SITL already built. Both launchers accept these overrides:

```bash
~/isaacsim/python.sh worlds/moving_plate.py \
  --px4-dir ~/PX4-Autopilot --px4-airframe gazebo-classic_iris
```

If Pegasus is not installed in Isaac's Python, add
`--pegasus-root ~/uas-pegasus-sim` or set `PEGASUS_PATH` to that checkout. The
extension directory `extensions/pegasus.simulator` is also accepted. Pegasus's
Python dependencies must already be installed, as for its standalone examples.
The field and target are built locally; the Iris asset comes from Pegasus.

## ROS2 environment setup

On this Ubuntu 22.04 workstation, use Isaac Sim's bundled ROS2 Humble libraries.
The `isaac_run` function in `~/.bashrc` already configures them. To launch directly
with `python.sh`, run this once in a fresh simulator terminal:

```bash
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="$HOME/isaacsim/exts/isaacsim.ros2.bridge/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
~/isaacsim/python.sh worlds/moving_plate.py --spawn-radius 0
```

Without that setup, the bridge can fail with `RMW was not loaded` and an empty
`AMENT_PREFIX_PATH` message. The configuration above was verified with the bridge
loading its internal `rclpy` and the world completing a headless run.

Use a separate terminal for external ROS2 nodes, sourcing
`/opt/ros/humble/setup.bash` there. Isaac Sim 5.1 uses Python 3.11, while the system
Humble installation uses Python 3.10; the bundled simulator libraries keep their
Python environments separate. See [NVIDIA's ROS2 setup instructions](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html).

The current worlds do not publish camera topics yet. The inspection camera is a
viewport camera; ROS2 image and camera-info publishers still need to be configured
when adding the Iris vision pipeline.

## Headless runs and snapshots

Both worlds accept `--headless`, `--steps`, and `--export-stage`:

```bash
~/isaacsim/python.sh worlds/moving_plate.py --headless --steps 1000 \
  --spawn-radius 0 --seed 42 --motion-radius 10 \
  --export-stage /tmp/moving-plate.usda
```

`--steps` exits after a finite number of world steps. The moving world prints
`WORLD_FINAL_STATE` with its elapsed simulation time and final plate pose.
`--export-stage` writes a USD snapshot of the initialized world. Launch the
Python world to run Pegasus dynamics, PX4, and target motion; a USD snapshot
does not contain the procedural motion program.
