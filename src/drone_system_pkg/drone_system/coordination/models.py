from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class DroneSnapshot:
    drone_id: str
    x: float
    y: float
    z: float = 0.0
    valid: bool = True


@dataclass(frozen=True)
class BirdSnapshot:
    index: int
    x: float
    y: float
    state: int
    z: float = 0.0


@dataclass
class AssignmentSet:
    by_drone: Dict[str, int]

    def get(self, drone_id: str) -> int:
        return int(self.by_drone.get(drone_id, -1))


@dataclass
class ModeDecision:
    mode: str
    chased: bool
    target_index: int
