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
    cruise_speed: float
    escape_speed: float
    recover_default_s: float
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

    def _inside_field(self, field_xy):
        return abs(self.x) <= field_xy and abs(self.y) <= field_xy

    def _random_target(self, field_xy, wander_margin):
        if self.profile == PROFILE_HARD:
            # Hard birds hug the square boundary: sample on an annulus
            # between (field_xy - margin) and (field_xy - small inset).
            ring_hi = max(1.5, field_xy - 0.6)
            ring_lo = max(1.0, field_xy - max(1.8, wander_margin + 0.2))
            rx = random.choice([-1.0, 1.0]) * random.uniform(ring_lo, ring_hi)
            ry = random.choice([-1.0, 1.0]) * random.uniform(ring_lo, ring_hi)
            return rx, ry
        m = max(1.0, field_xy - wander_margin)
        return random.uniform(-m, m), random.uniform(-m, m)

    def _pick_target(self, field_xy, wander_margin):
        self.target_x, self.target_y = self._random_target(field_xy, wander_margin)

    def _wander_velocity(self, field_xy, wander_margin, wander_arrival):
        dx = self.target_x - self.x
        dy = self.target_y - self.y
        if math.hypot(dx, dy) < wander_arrival:
            self._pick_target(field_xy, wander_margin)
            dx = self.target_x - self.x
            dy = self.target_y - self.y
        return _norm(dx, dy)

    def _flee_velocity(self, drone_pose_valid, drone_x, drone_y):
        if drone_pose_valid:
            # Run away from drone: velocity ∝ (bird - drone), normalized.
            vx = self.x - drone_x
            vy = self.y - drone_y
            if math.hypot(vx, vy) > 0.5:
                return _norm(vx, vy)
        # No drone pose: flee radially away from map origin.
        return _norm(self.x, self.y)

    def _to_center_velocity(self):
        if math.hypot(self.x, self.y) < 0.5:
            return 0.0, 0.0
        return _norm(-self.x, -self.y)

    def step(self, ctx: BirdStepContext):
        if self.target_x == self.x and self.target_y == self.y:
            self._pick_target(ctx.field_xy, ctx.wander_margin)

        if ctx.just_unchased:
            if self.profile == PROFILE_HARD and not self._inside_field(ctx.field_xy):
                self.outside_hold_s = HARD_OUTSIDE_HOLD_S
            else:
                self.recover_s = ctx.recover_default_s

        if ctx.chased:
            state = STATE_FLEE
            vx, vy = self._flee_velocity(ctx.drone_pose_valid, ctx.drone_x, ctx.drone_y)
            speed = ctx.escape_speed * (1.15 if self.profile == PROFILE_HARD else 1.0)
        elif self.profile == PROFILE_HARD and not self._inside_field(ctx.field_xy) and self.outside_hold_s > 0.0:
            state = STATE_ENTER
            vx, vy = 0.0, 0.0
            speed = 0.0
            self.outside_hold_s = max(0.0, self.outside_hold_s - ctx.dt)
        elif self.recover_s > 0.0:
            state = STATE_RECOVER
            vx, vy = self._to_center_velocity()
            speed = ctx.cruise_speed
            self.recover_s = max(0.0, self.recover_s - ctx.dt)
            if self.recover_s <= 0.0:
                self._pick_target(ctx.field_xy, ctx.wander_margin)
        elif not self._inside_field(ctx.field_xy):
            state = STATE_ENTER
            vx, vy = self._to_center_velocity()
            speed = ctx.cruise_speed
        else:
            state = STATE_WANDER
            vx, vy = self._wander_velocity(ctx.field_xy, ctx.wander_margin, ctx.wander_arrival)
            speed = ctx.cruise_speed

        # Euler step: position += v_hat * speed * dt
        self.x += vx * speed * ctx.dt
        self.y += vy * speed * ctx.dt

        clamp_xy = ctx.world_xy if self.profile == PROFILE_HARD else ctx.field_xy
        self.x = max(-clamp_xy, min(clamp_xy, self.x))
        self.y = max(-clamp_xy, min(clamp_xy, self.y))
        return state
