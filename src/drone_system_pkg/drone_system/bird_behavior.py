# Bird motion model shared by birds_node (wander, flee, recover, enter).

import math
import random
from dataclasses import dataclass

STATE_WANDER = 0
STATE_FLEE = 1
STATE_RECOVER = 2
STATE_ENTER = 3

PROFILE_EASY = "easy"
PROFILE_HARD = "hard"

HARD_OUTSIDE_HOLD_S = 3.0


def _norm(dx, dy):
    # Unit direction (dx, dy) / ||(dx, dy)||; fallback +X if length ~ 0.
    m = math.hypot(dx, dy)
    if m < 1e-6:
        return 1.0, 0.0
    return dx / m, dy / m


@dataclass
class BirdStepContext:
    dt: float
    chased: bool
    just_unchased: bool
    field_xy: float
    world_xy: float
    wall_margin: float
    hard_wall_ring: float
    cruise_speed: float
    escape_speed: float
    recover_default_s: float
    hard_reentry_s: float
    wander_margin: float
    wander_arrival: float
    drone_pose_valid: bool
    drone_x: float
    drone_y: float


class BirdAgent:
    def __init__(self, x, y, z, profile=PROFILE_EASY, intro_recover_s=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.profile = str(profile or PROFILE_EASY).lower()
        if self.profile not in (PROFILE_EASY, PROFILE_HARD):
            self.profile = PROFILE_EASY

        self.target_x = self.x
        self.target_y = self.y
        self.recover_s = max(0.0, float(intro_recover_s))
        self.outside_hold_s = 0.0
        self.outside_elapsed_s = 0.0
        self._coast_vx = 0.0
        self._coast_vy = 0.0
        self._coast_speed = 0.0

    def _inside_field(self, field_xy):
        return abs(self.x) <= field_xy and abs(self.y) <= field_xy

    @staticmethod
    def _roam_limit(world_xy, wall_margin):
        return max(1.0, world_xy - wall_margin)

    def _random_target(self, world_xy, wall_margin, wander_margin, hard_wall_ring):
        margin = max(wall_margin, wander_margin)
        roam = self._roam_limit(world_xy, margin)
        if self.profile == PROFILE_HARD:
            ring_hi = roam
            ring_lo = max(1.0, roam - hard_wall_ring)
            rx = random.choice([-1.0, 1.0]) * random.uniform(ring_lo, ring_hi)
            ry = random.choice([-1.0, 1.0]) * random.uniform(ring_lo, ring_hi)
            return rx, ry
        return random.uniform(-roam, roam), random.uniform(-roam, roam)

    def _pick_target(self, world_xy, wall_margin, wander_margin, hard_wall_ring):
        self.target_x, self.target_y = self._random_target(
            world_xy, wall_margin, wander_margin, hard_wall_ring
        )

    def _wander_velocity(
        self, world_xy, wall_margin, wander_margin, hard_wall_ring, wander_arrival
    ):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        if math.hypot(dx, dy) < wander_arrival:
            self._pick_target(world_xy, wall_margin, wander_margin, hard_wall_ring)
            dx = self.target_x - self.x
            dy = self.target_y - self.y
        return _norm(dx, dy)

    def _toward_closest_wall(self, field_xy):
        """Unit vector from bird toward the nearest field wall (±field_xy)."""
        # Signed distance to each wall; direction points from bird toward that wall.
        walls = (
            (field_xy - self.x, 1.0, 0.0),   # east  (+x)
            (self.x + field_xy, -1.0, 0.0),  # west  (-x)
            (field_xy - self.y, 0.0, 1.0),   # north (+y)
            (self.y + field_xy, 0.0, -1.0),  # south (-y)
        )
        inside = [(d, wx, wy) for d, wx, wy in walls if d > 0.0]
        if inside:
            _, wx, wy = min(inside, key=lambda t: t[0])
            return wx, wy
        # Already past the walls: keep pushing out through the nearest exit.
        _, wx, wy = min(walls, key=lambda t: t[0])
        return wx, wy

    def _flee_velocity(self, field_xy, drone_pose_valid, drone_x, drone_y):
        # Flee = toward closest wall + away from threat (drone), then normalize.
        wx, wy = self._toward_closest_wall(field_xy)
        if drone_pose_valid:
            tx, ty = self.x - drone_x, self.y - drone_y
            if math.hypot(tx, ty) > 0.5:
                tx, ty = _norm(tx, ty)
            else:
                tx, ty = wx, wy
        else:
            tx, ty = _norm(self.x, self.y) if math.hypot(self.x, self.y) > 0.5 else (wx, wy)
        return _norm(wx + tx, wy + ty)

    def _to_center_velocity(self):
        if math.hypot(self.x, self.y) < 0.5:
            return 0.0, 0.0
        return _norm(-self.x, -self.y)

    def step(self, ctx: BirdStepContext):
        inside_field = self._inside_field(ctx.field_xy)
        if inside_field:
            self.outside_elapsed_s = 0.0
        elif self.profile == PROFILE_HARD and not ctx.chased:
            self.outside_elapsed_s += ctx.dt

        if self.target_x == self.x and self.target_y == self.y:
            self._pick_target(
                ctx.world_xy, ctx.wall_margin, ctx.wander_margin, ctx.hard_wall_ring
            )

        if ctx.just_unchased:
            if not inside_field:
                # Keep current flee direction/speed for a short outside coasting window.
                self.outside_hold_s = HARD_OUTSIDE_HOLD_S
            else:
                self.recover_s = ctx.recover_default_s

        if ctx.chased:
            self.outside_elapsed_s = 0.0
            state = STATE_FLEE
            vx, vy = self._flee_velocity(
                ctx.field_xy, ctx.drone_pose_valid, ctx.drone_x, ctx.drone_y
            )
            speed = ctx.escape_speed * (1.15 if self.profile == PROFILE_HARD else 1.0)
            self._coast_vx, self._coast_vy, self._coast_speed = vx, vy, speed
        elif (
            self.profile == PROFILE_HARD
            and not inside_field
            and ctx.hard_reentry_s > 0.0
            and self.outside_elapsed_s >= ctx.hard_reentry_s
        ):
            state = STATE_ENTER
            vx, vy = self._to_center_velocity()
            speed = ctx.escape_speed
        elif not inside_field and self.outside_hold_s > 0.0:
            state = STATE_ENTER
            vx, vy = self._coast_vx, self._coast_vy
            speed = self._coast_speed
            self.outside_hold_s = max(0.0, self.outside_hold_s - ctx.dt)
        elif self.recover_s > 0.0:
            state = STATE_RECOVER
            vx, vy = self._to_center_velocity()
            speed = ctx.cruise_speed
            self.recover_s = max(0.0, self.recover_s - ctx.dt)
            if self.recover_s <= 0.0:
                self._pick_target(
                    ctx.world_xy, ctx.wall_margin, ctx.wander_margin, ctx.hard_wall_ring
                )
        else:
            # Wander anywhere in the world box (±limit_xy), including outside the fence.
            state = STATE_WANDER
            vx, vy = self._wander_velocity(
                ctx.world_xy,
                ctx.wall_margin,
                ctx.wander_margin,
                ctx.hard_wall_ring,
                ctx.wander_arrival,
            )
            speed = ctx.cruise_speed

        # Euler step: position += v_hat * speed * dt
        self.x += vx * speed * ctx.dt
        self.y += vy * speed * ctx.dt

        # Birds roam freely inside the world box (limit_xy). field_xy is only the fence.
        clamp_xy = ctx.world_xy
        self.x = max(-clamp_xy, min(clamp_xy, self.x))
        self.y = max(-clamp_xy, min(clamp_xy, self.y))
        return state
