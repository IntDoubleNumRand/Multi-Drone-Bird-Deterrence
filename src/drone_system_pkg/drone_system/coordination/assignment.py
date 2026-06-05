# Nearest-bird assignment (one bird per drone, no double-booking).

import math
from typing import Dict, Sequence, Set, Tuple

from drone_system.coordination.models import BirdSnapshot, DroneSnapshot

STATE_WANDER = 0
STATE_FLEE = 1


def detection_box_half_extents(box: dict) -> Tuple[float, float, float]:
    """Full box size (m) -> half-extents for axis-aligned detection volume."""
    if not isinstance(box, dict):
        return (1.0, 1.0, 1.0)
    return (
        max(0.1, float(box.get("x", 2.0))) / 2.0,
        max(0.1, float(box.get("y", 2.0))) / 2.0,
        max(0.1, float(box.get("z", 2.0))) / 2.0,
    )


def is_assignable_bird(bird: BirdSnapshot, field_xy: float, world_xy: float) -> bool:
    return _bird_in_play_area(bird, field_xy, world_xy)


def _bird_in_play_area(bird: BirdSnapshot, field_xy: float, world_xy: float) -> bool:
    if bird.state not in (STATE_WANDER, STATE_FLEE):
        return False
    # A bird outside the field fence has been chased off and should no longer
    # be assigned, even though it may continue to roam in the larger world box.
    return abs(bird.x) <= field_xy and abs(bird.y) <= field_xy


def is_proximate_bird(
    drone: DroneSnapshot,
    bird: BirdSnapshot,
    half_x: float,
    half_y: float,
    half_z: float,
    field_xy: float,
    world_xy: float,
    *,
    check_z: bool = False,
    z_above_only: bool = False,
    z_above_max: float = 0.0,
) -> bool:
    """Onboard sensor: drone inside proximity box around bird (default: XY only)."""
    if not _bird_in_play_area(bird, field_xy, world_xy) or not drone.valid:
        return False
    if abs(drone.x - bird.x) > half_x or abs(drone.y - bird.y) > half_y:
        return False
    if check_z:
        dz = drone.z - bird.z
        if z_above_only:
            max_up = float(z_above_max) if z_above_max > 0.0 else (2.0 * half_z)
            if dz < 0.0 or dz > max_up:
                return False
        elif abs(dz) > half_z:
            return False
    return True


def any_assignable_bird(
    birds: Sequence[BirdSnapshot],
    field_xy: float,
    world_xy: float,
) -> bool:
    return any(_bird_in_play_area(b, field_xy, world_xy) for b in birds)


def assign_nearest_in_field(
    drones: Sequence[DroneSnapshot],
    birds: Sequence[BirdSnapshot],
    field_xy: float,
    world_xy: float,
    *,
    require_valid_drones: bool = True,
) -> Dict[str, int]:
    """
    Central dispatcher: assign nearest active bird in the play area (no proximity gate).
    """
    active_drones = [d for d in drones if d.valid] if require_valid_drones else list(drones)
    lookup = {b.index: b for b in birds if _bird_in_play_area(b, field_xy, world_xy)}
    remaining: Set[int] = set(lookup.keys())
    result = {d.drone_id: -1 for d in drones}

    for drone in active_drones:
        best_idx = -1
        best_d = float("inf")
        for idx in remaining:
            b = lookup[idx]
            d = math.hypot(b.x - drone.x, b.y - drone.y)
            if d < best_d:
                best_d = d
                best_idx = idx
        result[drone.drone_id] = best_idx
        if best_idx >= 0:
            remaining.remove(best_idx)
    return result


def assign_nearest_detected(
    drones: Sequence[DroneSnapshot],
    birds: Sequence[BirdSnapshot],
    field_xy: float,
    world_xy: float,
    half_x: float,
    half_y: float,
    half_z: float,
    *,
    check_z: bool = False,
) -> Dict[str, int]:
    """Assign only birds inside the onboard proximity box (local fallback)."""
    valid_drones = [d for d in drones if d.valid]
    lookup = {b.index: b for b in birds if _bird_in_play_area(b, field_xy, world_xy)}
    remaining: Set[int] = set(lookup.keys())
    result = {d.drone_id: -1 for d in drones}

    for drone in valid_drones:
        best_idx = -1
        best_d = float("inf")
        for idx in remaining:
            b = lookup[idx]
            if not is_proximate_bird(
                drone,
                b,
                half_x,
                half_y,
                half_z,
                field_xy,
                world_xy,
                check_z=check_z,
            ):
                continue
            d = math.hypot(b.x - drone.x, b.y - drone.y)
            if d < best_d:
                best_d = d
                best_idx = idx
        result[drone.drone_id] = best_idx
        if best_idx >= 0:
            remaining.remove(best_idx)
    return result
