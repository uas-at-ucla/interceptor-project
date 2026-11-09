"""Shared Isaac/Pegasus/PX4 application setup; launch a named world instead."""

import argparse
import json
import math
import os
from pathlib import Path
import random
import sys

from .scene import (
    FIELD_SIZE_M, IRIS_SPAWN_M, PLATE_BOUND_RADIUS_M, PLATE_RADIUS_M, SEARCH_RADIUS_M,
    build_world, sample_plate,
)
from .motion import RandomPlateMotion


def plate_height(value):
    height = float(value)
    if not math.isfinite(height) or height <= PLATE_RADIUS_M:
        raise argparse.ArgumentTypeError(f"must be finite and greater than {PLATE_RADIUS_M} meters")
    return height


def positive_steps(value):
    steps = int(value)
    if steps <= 0:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return steps


def positive_float(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def nonnegative_float(value):
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise argparse.ArgumentTypeError("must be finite and nonnegative")
    return number


def parse_args(moving=False):
    name = "moving_plate" if moving else "static_plate"
    parser = argparse.ArgumentParser(description=f"Launch the {name} world with an Iris and PX4 SITL.")
    parser.add_argument("--seed", type=int,
                        help="placement and motion seed (default: a new random seed each launch)")
    parser.add_argument("--plate-height", type=plate_height, default=2.0,
                        help="plate center above the ground, in meters (default: 2)")
    parser.add_argument("--spawn-radius", type=nonnegative_float,
                        help="plate's initial horizontal spawn radius in meters "
                             "(default: 152.4 = 500 feet); 0 places it above Iris spawn")
    parser.add_argument("--headless", action="store_true", help="run without the Isaac Sim window")
    parser.add_argument("--steps", type=positive_steps, help="exit after this many world steps")
    parser.add_argument("--px4-dir", type=Path, help="override Pegasus's PX4-Autopilot directory")
    parser.add_argument("--px4-airframe", help="override Pegasus's PX4 airframe, e.g. gazebo-classic_iris")
    parser.add_argument("--pegasus-root", type=Path, default=os.environ.get("PEGASUS_PATH"),
                        help="Pegasus/isaac-uav checkout, if it is not installed in Isaac's Python; "
                             "also accepts the extension directory (or set PEGASUS_PATH)")
    parser.add_argument("--export-stage", type=Path, help="optional USD snapshot of the initialized world")
    if moving:
        parser.add_argument("--speed", type=positive_float, default=6.0,
                            help="maximum plate translation speed in m/s (default: 6)")
        parser.add_argument("--motion-radius", type=positive_float,
                            help="horizontal roaming radius around Iris spawn in meters "
                                 "(default: 152.4, increased to fit a larger --spawn-radius)")
        parser.add_argument("--height-variation", type=nonnegative_float, default=0.75,
                            help="random height variation above/below --plate-height in meters (default: 0.75)")
        parser.add_argument("--turn-interval", type=positive_float, default=1.5,
                            help="average seconds between random direction changes (default: 1.5)")
    args = parser.parse_args()
    if moving:
        if args.motion_radius is None:
            args.motion_radius = max(SEARCH_RADIUS_M, args.spawn_radius or 0.0)
        if args.spawn_radius is None:
            args.spawn_radius = min(SEARCH_RADIUS_M, args.motion_radius)
        if args.spawn_radius > args.motion_radius:
            parser.error("--spawn-radius must be no larger than --motion-radius")
        if args.plate_height - args.height_variation <= PLATE_BOUND_RADIUS_M:
            parser.error("--plate-height minus --height-variation must keep the tilted plate above the ground")
    elif args.spawn_radius is None:
        args.spawn_radius = SEARCH_RADIUS_M
    if args.pegasus_root:
        root = args.pegasus_root.expanduser().resolve()
        extension = root / "extensions" / "pegasus.simulator"
        if not (extension / "pegasus").is_dir():
            extension = root
        if not (extension / "pegasus" / "simulator").is_dir():
            parser.error(f"no pegasus/simulator package found under {root}")
        sys.path.insert(0, str(extension))
    return args


def main(*, moving=False):
    args = parse_args(moving)
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(2**32)
    placement = sample_plate(seed, args.plate_height, args.spawn_radius)
    field_radius = max(args.spawn_radius, args.motion_radius if moving else 0.0)
    field_size = max(FIELD_SIZE_M, 2.0 * (field_radius + 10.0))
    motion = RandomPlateMotion(
        seed, placement, speed_m_s=args.speed, radius_m=args.motion_radius,
        height_variation_m=args.height_variation,
        turn_interval_s=args.turn_interval,
    ) if moving else None

    # Isaac must start before importing omni, USD, or the Pegasus package.
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({
        "headless": args.headless, "width": 1280, "height": 720, "display_options": 0,
        "fast_shutdown": True,
    })
    timeline = None
    try:
        import omni.timeline
        from isaacsim.core.api import World
        from isaacsim.core.utils.viewports import set_camera_view
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig,
        )
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
        from pegasus.simulator.params import ROBOTS

        timeline = omni.timeline.get_timeline_interface()
        pg = PegasusInterface()
        px4_dir = (args.px4_dir or Path(pg.px4_path)).expanduser().resolve()
        px4_binary = px4_dir / "build/px4_sitl_default/bin/px4"
        if not px4_binary.is_file():
            raise FileNotFoundError(f"Build PX4 SITL first; missing {px4_binary}")

        # Pegasus vehicles look up this World through the interface singleton.
        pg._world = World(**{**pg._world_settings, "stage_units_in_meters": 1.0})
        world = pg.world
        plate_path = build_world(world.stage, placement, moving=moving, field_size_m=field_size)

        config = MultirotorConfig()
        config.backends = [PX4MavlinkBackend(PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": str(px4_dir),
            "px4_vehicle_model": args.px4_airframe or pg.px4_default_airframe,
        }))]

        Multirotor("/World/iris", ROBOTS["Iris"], 0, list(IRIS_SPAWN_M),
                   [0.0, 0.0, 0.0, 1.0], config=config)
        world.reset()
        if motion is not None:
            from pxr import Gf
            plate = world.stage.GetPrimAtPath(plate_path)
            translate = plate.GetAttribute("xformOp:translate")
            rotate = plate.GetAttribute("xformOp:rotateXYZ")

            def apply_motion_pose():
                translate.Set(Gf.Vec3d(*motion.position))
                rotate.Set(Gf.Vec3f(*motion.rotation_deg))

            def advance_plate(dt):
                motion.advance(dt)
                apply_motion_pose()

            def reset_plate(event):
                if event.type == int(omni.timeline.TimelineEventType.STOP):
                    motion.reset()
                    apply_motion_pose()

            world.add_physics_callback("plate_motion", advance_plate)
            world.add_timeline_callback("plate_motion_reset", reset_plate)
        if args.spawn_radius == 0:
            x, y, z = placement.position
            set_camera_view(eye=[x + 1.6, y, z + 0.2], target=list(placement.position))
        else:
            set_camera_view(eye=[2.5, -3.5, 1.8], target=list(IRIS_SPAWN_M))
        if args.export_stage:
            destination = args.export_stage.expanduser().resolve()
            destination.parent.mkdir(parents=True, exist_ok=True)
            world.stage.Export(str(destination))

        print("WORLD_METADATA " + json.dumps({
            "world": "moving_plate" if moving else "static_plate", "seed": seed, "backend": "px4",
            "iris_spawn_m": IRIS_SPAWN_M, "plate_center_m": placement.position,
            "plate_distance_m": placement.distance_m, "search_radius_m": args.spawn_radius,
            "spawn_radius_m": args.spawn_radius, "field_size_m": field_size,
            "plate_prim": str(plate_path), "marker": "red_square",
            "motion": {
                "max_speed_m_s": args.speed, "radius_m": args.motion_radius,
                "height_variation_m": args.height_variation,
                "turn_interval_s": args.turn_interval,
            } if moving else None,
        }), flush=True)

        timeline.play()
        steps = 0
        while simulation_app.is_running() and (args.steps is None or steps < args.steps):
            world.step(render=True)
            steps += 1
        print(f"Completed {steps} world steps.", flush=True)
        if motion is not None:
            print("WORLD_FINAL_STATE " + json.dumps({
                "motion_time_s": motion.time_s,
                "plate_center_m": tuple(translate.Get()),
                "plate_rotation_deg": tuple(rotate.Get()),
            }), flush=True)
    except Exception:
        # Kit's fast shutdown exits before Python can print an unhandled exception.
        # Report it now and set Kit's exit code before closing the application.
        import traceback
        import omni.kit.app
        traceback.print_exc()
        omni.kit.app.get_app().post_quit(1)
        raise
    finally:
        if timeline is not None:
            timeline.stop()
        simulation_app.close(wait_for_replicator=False)
