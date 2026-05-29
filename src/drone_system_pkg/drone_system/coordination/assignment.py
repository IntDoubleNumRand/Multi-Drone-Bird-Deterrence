# Nearest-bird assignment (one bird per drone, no double-booking).

import math
from typing import Dict, Sequence, Set

from drone_system.coordination.models import BirdSnapshot, DroneSnapshot

STATE_WANDER = 0
STATE_FLEE = 1


def is_active_bird(bird: BirdSnapshot, field_xy: float) -> bool:
    inside = abs(bird.x) <= field_xy and abs(bird.y) <= field_xy
    return inside and bird.state in (STATE_WANDER, STATE_FLEE)


def assign_nearest_unique(
    drones: Sequence[DroneSnapshot],
    birds: Sequence[BirdSnapshot],
    field_xy: float,
) -> Dict[str, int]:
    """
    Greedy assignment: each valid drone gets the closest unclaimed active bird.

    Distance is Euclidean in the map plane: hypot(bx - dx, by - dy).
    """
    valid_drones = [d for d in drones if d.valid]
    active_birds = [b for b in birds if is_active_bird(b, field_xy)]
    remaining: Set[int] = set(b.index for b in active_birds)
    lookup = {b.index: b for b in active_birds}
    result = {d.drone_id: -1 for d in drones}

    for drone in valid_drones:
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
