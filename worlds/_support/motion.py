"""Seeded target motion in simulation time; no vision or vehicle control logic."""

import math
import random

from .scene import IRIS_SPAWN_M, PLATE_BOUND_RADIUS_M, SEARCH_RADIUS_M, PlatePlacement


class RandomPlateMotion:
    """Frequent random turns with bounded position, altitude, and speed.

    Quintic easing brings velocity and acceleration to zero at each waypoint.
    Interpolation stays inside the convex search disk and altitude interval.
    Nearby goals and a minimum heading change prevent long straight runs.
    The peak easing derivative is 1.875, used to enforce the speed limit.
    """

    def __init__(self, seed: int, initial: PlatePlacement, *, speed_m_s=6.0,
                 radius_m=SEARCH_RADIUS_M, height_variation_m=0.75, turn_interval_s=1.5):
        if not math.isfinite(speed_m_s) or speed_m_s <= 0:
            raise ValueError("Speed must be finite and greater than zero.")
        if not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError("Motion radius must be finite and greater than zero.")
        if not math.isfinite(height_variation_m) or height_variation_m < 0:
            raise ValueError("Height variation must be finite and nonnegative.")
        if not math.isfinite(turn_interval_s) or turn_interval_s <= 0:
            raise ValueError("Turn interval must be finite and greater than zero.")
        if not all(math.isfinite(v) for v in (*initial.position, initial.yaw_deg)):
            raise ValueError("Initial pose must be finite.")
        if initial.distance_m > radius_m + 1e-9:
            raise ValueError("Initial plate position must be inside the motion radius.")
        if initial.position[2] - height_variation_m <= PLATE_BOUND_RADIUS_M:
            raise ValueError("The entire altitude range must keep the plate above the ground.")
        self.seed = seed
        self.initial = initial
        self.speed_m_s = speed_m_s
        self.radius_m = radius_m
        self.height_variation_m = height_variation_m
        self.turn_interval_s = turn_interval_s
        self.reset()

    def reset(self):
        """Restart the same trajectory; called when the simulation is stopped/reset."""
        self._rng = random.Random(f"plate-motion:{self.seed}")
        self.time_s = 0.0
        self.position = self.initial.position
        self._yaw_deg = self.initial.yaw_deg
        self._segment_start = self.position
        self._segment_time = 0.0
        self._last_heading = None
        self._choose_goal()

    def _choose_goal(self):
        self._segment_duration = self.turn_interval_s * self._rng.uniform(0.6, 1.4)
        # Restrict each leg's length, rather than crossing the entire field before
        # turning. This also bounds peak 3D speed under quintic easing.
        reach = min(self.radius_m, self.speed_m_s * self._segment_duration / 1.875)
        x, y, z = self._segment_start
        low = self.initial.position[2] - self.height_variation_m
        high = self.initial.position[2] + self.height_variation_m
        for _ in range(64):
            heading = self._rng.uniform(-math.pi, math.pi)
            if self._last_heading is not None:
                turn = (heading - self._last_heading + math.pi) % math.tau - math.pi
                if abs(turn) < math.pi / 3:
                    continue
            distance = reach * self._rng.uniform(0.4, 1.0)
            vertical = self._rng.uniform(-0.5, 0.5)
            horizontal = distance * math.sqrt(1.0 - vertical * vertical)
            goal_x = x + horizontal * math.cos(heading)
            goal_y = y + horizontal * math.sin(heading)
            if math.hypot(goal_x - IRIS_SPAWN_M[0], goal_y - IRIS_SPAWN_M[1]) > self.radius_m:
                continue
            self._segment_goal = (goal_x, goal_y, max(low, min(high, z + distance * vertical)))
            self._last_heading = heading
            return

        # If rejected near the boundary, take a short inward leg before trying
        # another random turn. This remains inside the disk and speed limit.
        heading = math.atan2(IRIS_SPAWN_M[1] - y, IRIS_SPAWN_M[0] - x)
        distance = min(reach * 0.5, math.hypot(x - IRIS_SPAWN_M[0], y - IRIS_SPAWN_M[1]))
        self._segment_goal = (x + distance * math.cos(heading), y + distance * math.sin(heading), z)
        self._last_heading = heading

    @property
    def rotation_deg(self):
        # Keep the marker broadly facing spawn, with gentle frisbee-like rocking.
        return (90.0 + 12.0 * math.sin(0.8 * self.time_s),
                8.0 * math.sin(1.1 * self.time_s), self._yaw_deg)

    def advance(self, dt: float):
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("Time step must be finite and nonnegative.")
        self.time_s += dt
        remaining = dt
        while remaining > 0:
            step = min(remaining, self._segment_duration - self._segment_time)
            self._segment_time += step
            remaining -= step
            t = self._segment_time / self._segment_duration
            eased = t * t * t * (10.0 + t * (-15.0 + 6.0 * t))
            self.position = tuple(a + (b - a) * eased
                                  for a, b in zip(self._segment_start, self._segment_goal))
            # Limit turning rate, including the initially ambiguous heading at spawn.
            x = IRIS_SPAWN_M[0] - self.position[0]
            y = IRIS_SPAWN_M[1] - self.position[1]
            if math.hypot(x, y) > 1e-6:
                target_yaw = math.degrees(math.atan2(y, x)) + 90.0
                difference = (target_yaw - self._yaw_deg + 180.0) % 360.0 - 180.0
                self._yaw_deg += max(-60.0 * step, min(60.0 * step, difference))
            if self._segment_time >= self._segment_duration:
                self.position = self._segment_goal
                self._segment_start = self.position
                self._segment_time = 0.0
                self._choose_goal()
