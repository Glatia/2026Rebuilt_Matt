"""
Inspired by https://github.com/hammerheads5000/FuelSim v1.0.3
"""
# pylint: skip-file
import math
import random
from dataclasses import dataclass, field
from typing import Callable, ClassVar, Optional

import numpy as np
from pykit.logger import Logger
from wpilib import RobotBase
from wpimath.geometry import (
    Pose2d, Pose3d, Rotation3d, Transform3d,
    Translation2d, Translation3d,
)
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (
    kilograms, kilograms_per_cubic_meter, meters,
    meters_per_second, meters_per_second_squared, radians, seconds,
)

if RobotBase.isSimulation():
    import gc
    gc.set_threshold(10000, 10, 10)

# Constants
_PERIOD: seconds = 0.02
_GRAVITY: meters_per_second_squared = -9.81
_AIR_DENSITY: kilograms_per_cubic_meter = 1.2041
_FIELD_COR = math.sqrt(22 / 51.5)
_FUEL_COR = 0.5
_NET_COR = 0.2
_ROBOT_COR = 0.1
_FUEL_RADIUS: meters = 0.075
_FIELD_LENGTH: meters = 16.51
_FIELD_WIDTH: meters = 8.04
_TRENCH_WIDTH: meters = 1.265
_TRENCH_BLOCK_WIDTH: meters = 0.305
_TRENCH_HEIGHT: meters = 0.565
_TRENCH_BAR_HEIGHT: meters = 0.102
_TRENCH_BAR_WIDTH: meters = 0.152
_FRICTION = 0.1
_FUEL_MASS: kilograms = 0.448 * 0.45392
_FUEL_CROSS_AREA = math.pi * _FUEL_RADIUS ** 2
_DRAG_COF = 0.47
_DRAG_FORCE_FACTOR = 0.5 * _AIR_DENSITY * _DRAG_COF * _FUEL_CROSS_AREA

# Pre-computed stuff
_DRAG_OVER_MASS = _DRAG_FORCE_FACTOR / _FUEL_MASS
_FIELD_COR1 = 1.0 + _FIELD_COR
_FUEL_COR1 = 1.0 + _FUEL_COR
_ROBOT_COR1 = 1.0 + _ROBOT_COR
_FUEL_DIAM = _FUEL_RADIUS * 2.0
_FUEL_DIAM_SQ = _FUEL_DIAM ** 2

# Line checks
_FIELD_XZ_LINES: tuple[tuple[Translation3d, Translation3d], ...] = (
    (Translation3d(0, 0, 0), Translation3d(_FIELD_LENGTH, _FIELD_WIDTH, 0)),
    (Translation3d(3.96, 1.57, 0),
     Translation3d(4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),
    (Translation3d(3.96, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(4.61, _FIELD_WIDTH - 1.57, 0.165)),
    (Translation3d(4.61, 1.57, 0.165),
     Translation3d(5.18, _FIELD_WIDTH / 2 - 0.60, 0)),
    (Translation3d(4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(5.18, _FIELD_WIDTH - 1.57, 0)),
    (Translation3d(_FIELD_LENGTH - 5.18, 1.57, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 - 0.60, 0.165)),
    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH - 1.57, 0.165)),
    (Translation3d(_FIELD_LENGTH - 4.61, 1.57, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH / 2 - 0.60, 0)),
    (Translation3d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(_FIELD_LENGTH - 3.96, _FIELD_WIDTH - 1.57, 0)),
    (Translation3d(3.96, _TRENCH_WIDTH, _TRENCH_HEIGHT),
     Translation3d(5.18, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT)),
    (Translation3d(3.96, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT),
     Translation3d(
         5.18,
         _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
     )),
    (Translation3d(_FIELD_LENGTH - 5.18, _TRENCH_WIDTH, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
     )),
    (Translation3d(_FIELD_LENGTH - 5.18, _FIELD_WIDTH - 1.57, _TRENCH_HEIGHT),
     Translation3d(
         _FIELD_LENGTH - 3.96,
         _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT
     )),
    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2,
        0,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ),
     Translation3d(
         4.61 + _TRENCH_BAR_WIDTH / 2,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
     )),
    (Translation3d(
        4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ),
     Translation3d(
         4.61 + _TRENCH_BAR_WIDTH / 2,
         _FIELD_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
     )),
    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        0,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ),
     Translation3d(
         _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
         _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
     )),
    (Translation3d(
        _FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
        _FIELD_WIDTH - 1.57,
        _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
    ),
     Translation3d(
         _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2,
         _FIELD_WIDTH,
         _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT
     )),
)

# Convert lines to numpy array
_NP_LINES: list[tuple[np.ndarray, np.ndarray]] = [
    (np.array([s.x, s.y, s.z]), np.array([e.x, e.y, e.z]))
    for s, e in _FIELD_XZ_LINES
]

_LINE_BOUNDS: list[tuple[float, float, float, float]] = [
    (min(s.x, e.x), max(s.x, e.x), min(s.y, e.y), max(s.y, e.y))
    for s, e in _FIELD_XZ_LINES
]

# x0, x1, y0, y1, z0, z1
_TRENCH_RECTS: tuple[tuple[float, float, float, float, float, float], ...] = (
    (3.96, 5.18, _TRENCH_WIDTH, _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, 0.0,
     _TRENCH_HEIGHT),
    (3.96, 5.18, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (_FIELD_LENGTH - 5.18, _FIELD_LENGTH - 3.96, _TRENCH_WIDTH,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (_FIELD_LENGTH - 5.18, _FIELD_LENGTH - 3.96, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH - 1.57 + _TRENCH_BLOCK_WIDTH, 0.0, _TRENCH_HEIGHT),
    (4.61 - _TRENCH_BAR_WIDTH / 2, 4.61 + _TRENCH_BAR_WIDTH / 2, 0.0,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (4.61 - _TRENCH_BAR_WIDTH / 2, 4.61 + _TRENCH_BAR_WIDTH / 2,
     _FIELD_WIDTH - 1.57, _FIELD_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (_FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
     _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2, 0.0,
     _TRENCH_WIDTH + _TRENCH_BLOCK_WIDTH, _TRENCH_HEIGHT,
     _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
    (_FIELD_LENGTH - 4.61 - _TRENCH_BAR_WIDTH / 2,
     _FIELD_LENGTH - 4.61 + _TRENCH_BAR_WIDTH / 2, _FIELD_WIDTH - 1.57,
     _FIELD_WIDTH, _TRENCH_HEIGHT, _TRENCH_HEIGHT + _TRENCH_BAR_HEIGHT),
)

# Bounding box for the trench
_TRENCH_X_MIN = 3.96 - _FUEL_RADIUS
_TRENCH_X_MAX = _FIELD_LENGTH - 3.96 + _FUEL_RADIUS
_TRENCH_X_LEFT_MAX = 5.18 + _FUEL_RADIUS
_TRENCH_X_RIGHT_MIN = _FIELD_LENGTH - 5.18 - _FUEL_RADIUS


@dataclass
class Hub:
    """Handles hub interactions with fuel and collisions."""
    center: Translation2d
    exit: Translation3d
    exit_vel_x_mult: int

    _score: int = field(default=0, init=False, repr=False)

    ENTRY_HEIGHT: ClassVar[float] = 1.83
    ENTRY_RADIUS: ClassVar[float] = 0.56
    _SIDE: ClassVar[float] = 1.2
    _NET_HEIGHT_MAX: ClassVar[float] = 3.057
    _NET_HEIGHT_MIN: ClassVar[float] = 1.5
    _NET_OFFSET: ClassVar[float] = _SIDE / 2 + 0.261
    _NET_WIDTH: ClassVar[float] = 1.484

    def __post_init__(self) -> None:
        self.cx: float = self.center.x
        self.cy: float = self.center.y
        self.net_x: float = self.cx + self._NET_OFFSET * self.exit_vel_x_mult

    def did_score(
        self, pos: np.ndarray, vel: np.ndarray, i: int, dt: float
    ) -> bool:
        """Check if fuel entered the hub, if so raise score and dispense."""
        px, py, pz = pos[i, 0], pos[i, 1], pos[i, 2]
        dx, dy = px - self.cx, py - self.cy
        dist2d = math.sqrt(dx * dx + dy * dy)
        prev_z = pz - vel[i, 2] * dt
        if not (dist2d <= self.ENTRY_RADIUS and
                pz <= self.ENTRY_HEIGHT < prev_z):
            return False

        pos[i] = [self.exit.x, self.exit.y, self.exit.z]
        vel[i] = [
            self.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
            random.uniform(-1, 1),
            0.0,
        ]
        self._score += 1
        return True

    def collide_side(self, pos: np.ndarray, vel: np.ndarray, i: int) -> None:
        """AABB collision against the hub body."""
        _rect_collide_np(
            pos, vel, i,
            self.cx - self._SIDE / 2, self.cx + self._SIDE / 2,
            self.cy - self._SIDE / 2, self.cy + self._SIDE / 2,
            0.0, self.ENTRY_HEIGHT - 0.1,
        )

    def collide_net(self, pos: np.ndarray, vel: np.ndarray, i: int) -> None:
        """Inelastic bounce off the hub net."""
        pz = pos[i, 2]
        if pz > self._NET_HEIGHT_MAX or pz < self._NET_HEIGHT_MIN:
            return
        py = pos[i, 1]
        if (py > self.cy + self._NET_WIDTH / 2 or
                py < self.cy - self._NET_WIDTH / 2):
            return
        px = pos[i, 0]
        nx = self.net_x
        offset = max(0.0, nx - (px - _FUEL_RADIUS)) if px > nx else \
            min(0.0, nx - (px + _FUEL_RADIUS))
        if offset == 0.0:
            return
        pos[i, 0] += offset
        vel[i, 0] = -vel[i, 0] * _NET_COR
        vel[i, 1] *= _NET_COR

    @property
    def score(self) -> int:
        """Total fuel scored"""
        return self._score

    @score.setter
    def score(self, value: int) -> None:
        self._score = value

    def reset_score(self) -> None:
        """Reset score to 0"""
        self._score = 0


BLUE_HUB = Hub(
    Translation2d(4.61, _FIELD_WIDTH / 2),
    Translation3d(5.3, _FIELD_WIDTH / 2, 0.89),
    1,
)
RED_HUB = Hub(
    Translation2d(_FIELD_LENGTH - 4.61, _FIELD_WIDTH / 2),
    Translation3d(_FIELD_LENGTH - 5.3, _FIELD_WIDTH / 2, 0.89),
    -1,
)

_HUB_CLOSE_SQ = (Hub.ENTRY_RADIUS + 1.0) ** 2


def _rect_collide_np(
    pos: np.ndarray, vel: np.ndarray, i: int,
    x0: float, x1: float, y0: float, y1: float, z0: float, z1: float,
) -> None:
    """In-place AABB collision for a single fuel (index i)."""
    vx, vy, vz = vel[i, 0], vel[i, 1], vel[i, 2]
    if vx * vx + vy * vy + vz * vz < 1e-12:
        return
    px, py, pz = pos[i, 0], pos[i, 1], pos[i, 2]
    if pz > z1 + _FUEL_RADIUS or pz < z0 - _FUEL_RADIUS:
        return

    d_left = x0 - _FUEL_RADIUS - px
    d_right = px - x1 - _FUEL_RADIUS
    d_top = py - y1 - _FUEL_RADIUS
    d_bot = y0 - _FUEL_RADIUS - py

    if d_left > 0 or d_right > 0 or d_top > 0 or d_bot > 0:
        return

    if px < x0 or (d_left >= d_right and d_left >= d_top and d_left >= d_bot):
        pos[i, 0] += d_left
        vel[i, 0] += -_FIELD_COR1 * vx
    elif px >= x1 or (
            d_right >= d_left and d_right >= d_top and d_right >= d_bot):
        pos[i, 0] -= d_right
        vel[i, 0] += -_FIELD_COR1 * vx
    elif py > y1 or (d_top >= d_left and d_top >= d_right and d_top >= d_bot):
        pos[i, 1] -= d_top
        vel[i, 1] += -_FIELD_COR1 * vy
    else:
        pos[i, 1] += d_bot
        vel[i, 1] += -_FIELD_COR1 * vy


def _xz_line_collide_np(
    pos: np.ndarray, vel: np.ndarray, i: int,
    start: np.ndarray, end: np.ndarray,
) -> None:
    """In-place XZ-plane line collision for a single fuel."""
    py = pos[i, 1]
    if py < start[1] or py > end[1]:
        return

    sx, sz = start[0], start[2]
    ex, ez = end[0], end[2]
    px, pz = pos[i, 0], pos[i, 2]

    lvx, lvz = ex - sx, ez - sz
    norm_sq = lvx * lvx + lvz * lvz
    if norm_sq < 1e-12:
        return

    t = ((px - sx) * lvx + (pz - sz) * lvz) / norm_sq
    if t < 0.0 or t > 1.0:
        return

    proj_x = sx + t * lvx
    proj_z = sz + t * lvz

    ddx = px - proj_x
    ddz = pz - proj_z
    dist_sq = ddx * ddx + ddz * ddz
    if dist_sq > _FUEL_RADIUS * _FUEL_RADIUS:
        return

    dist = math.sqrt(dist_sq) if dist_sq > 1e-12 else 1e-6
    seg_len = math.sqrt(norm_sq)
    nx = -lvz / seg_len
    nz = lvx / seg_len

    overlap = _FUEL_RADIUS - dist
    pos[i, 0] += nx * overlap
    pos[i, 2] += nz * overlap

    vx, vz = vel[i, 0], vel[i, 2]
    vdotn = vx * nx + vz * nz
    if vdotn >= 0:
        return
    vel[i, 0] -= (1.0 + _FIELD_COR) * vdotn * nx
    vel[i, 2] -= (1.0 + _FIELD_COR) * vdotn * nz


@dataclass
class SimIntake:
    """Robot intake region in robot-relative coordinates."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    able_to_intake: Callable[[], bool] = field(default=lambda: True)
    callback: Callable[[], None] = field(default=lambda: None)


# Grid constants
_CELL_SIZE = _FUEL_RADIUS * 2.2
_GRID_COLS = math.ceil(_FIELD_LENGTH / _CELL_SIZE)
_GRID_ROWS = math.ceil(_FIELD_WIDTH / _CELL_SIZE)
_SLEEP_THRESHOLD = 10
_SLEEP_SPEED_SQ = 1e-6  # (m/s)²


def _collide_trench(pos: np.ndarray, vel: np.ndarray, i: int) -> None:
    """Check for trench collisions for fuel at index i"""
    px = pos[i, 0]
    if not (_TRENCH_X_MIN <= px <= _TRENCH_X_LEFT_MAX or
            _TRENCH_X_RIGHT_MIN <= px <= _TRENCH_X_MAX):
        return
    for x0, x1, y0, y1, z0, z1 in _TRENCH_RECTS:
        _rect_collide_np(pos, vel, i, x0, x1, y0, y1, z0, z1)


def _collide_edges(pos: np.ndarray, vel: np.ndarray, i: int) -> None:
    """Check for edge collisions for fuel at index i"""
    px, py = pos[i, 0], pos[i, 1]
    vx, vy = vel[i, 0], vel[i, 1]
    if px < _FUEL_RADIUS and vx < 0:
        pos[i, 0] = _FUEL_RADIUS
        vel[i, 0] += -(1.0 + _FIELD_COR) * vx
    elif px > _FIELD_LENGTH - _FUEL_RADIUS and vx > 0:
        pos[i, 0] = _FIELD_LENGTH - _FUEL_RADIUS
        vel[i, 0] += -(1.0 + _FIELD_COR) * vx
    if py < _FUEL_RADIUS and vy < 0:
        pos[i, 1] = _FUEL_RADIUS
        vel[i, 1] += -(1.0 + _FIELD_COR) * vy
    elif py > _FIELD_WIDTH - _FUEL_RADIUS and vy > 0:
        pos[i, 1] = _FIELD_WIDTH - _FUEL_RADIUS
        vel[i, 1] += -(1.0 + _FIELD_COR) * vy


class FuelSim:
    """
    Vectorized fuel simulator.

    Internal state:
        _pos : np.ndarray  shape (capacity, 3)  float64  [x, y, z]
        _vel : np.ndarray  shape (capacity, 3)  float64  [vx, vy, vz]
        _alive : np.ndarray  shape (capacity,)  bool
        _sleep : np.ndarray  shape (capacity,)  uint8   sleep counter
        _n : int  — number of allocated slots (may include dead slots)
    """

    __slots__ = (
        "_pos", "_vel", "_alive", "_sleep", "_n", "_cap",
        "running", "simulate_air_resistance", "subticks", "intakes",
        "_table_key", "robot_pose_supplier", "robot_speeds_supplier",
        "robot_width", "robot_length", "bumper_height",
        "_broad_r_sq",
        "_grid", "_grid_pool", "_grid_active_keys",
        "_pair_gen", "_current_gen",
    )

    def __init__(self, table_key: str = "Fuel Simulation") -> None:
        cap = 600
        self._pos = np.zeros((cap, 3), dtype=np.float64)
        self._vel = np.zeros((cap, 3), dtype=np.float64)
        self._alive = np.zeros(cap, dtype=bool)
        self._sleep = np.zeros(cap, dtype=np.uint8)
        self._n = 0
        self._cap = cap

        self._grid: dict[tuple[int, int], list[int]] = {}
        self._grid_pool: list[list[int]] = []
        self._grid_active_keys: list[tuple[int, int]] = []
        self._pair_gen = np.zeros((cap, cap), dtype=np.uint32)
        self._current_gen: int = 0

        self.running: bool = False
        self.simulate_air_resistance: bool = False
        self.subticks: int = 5
        self.intakes: list[SimIntake] = []
        self._table_key = table_key

        self.robot_pose_supplier: Optional[Callable] = None
        self.robot_speeds_supplier: Optional[Callable] = None
        self.robot_width: float = 0.0
        self.robot_length: float = 0.0
        self.bumper_height: float = 0.0
        self._broad_r_sq: float = 0.0

    def _ensure_capacity(self, extra: int) -> None:
        needed = self._n + extra
        if needed <= self._cap:
            return
        new_cap = max(needed, self._cap * 2)

        new_pos = np.zeros((new_cap, 3), dtype=np.float64)
        new_pos[:self._cap] = self._pos
        self._pos = new_pos

        new_vel = np.zeros((new_cap, 3), dtype=np.float64)
        new_vel[:self._cap] = self._vel
        self._vel = new_vel

        new_alive = np.zeros(new_cap, dtype=bool)
        new_alive[:self._cap] = self._alive[:self._cap]
        self._alive = new_alive

        new_sleep = np.zeros(new_cap, dtype=np.uint8)
        new_sleep[:self._cap] = self._sleep[:self._cap]
        self._sleep = new_sleep

        if new_cap > self._pair_gen.shape[0]:
            new_gen = np.zeros((new_cap, new_cap), dtype=np.uint32)
            new_gen[:self._cap, :self._cap] = self._pair_gen
            self._pair_gen = new_gen

        self._cap = new_cap


    def _add_fuel(self, px: float, py: float, pz: float,
                  vx: float = 0.0, vy: float = 0.0, vz: float = 0.0
                  ) -> int:
        """Add new fuel to the array"""
        self._ensure_capacity(1)
        i = self._n
        self._pos[i] = [px, py, pz]
        self._vel[i] = [vx, vy, vz]
        self._alive[i] = True
        self._sleep[i] = 0
        self._n += 1
        return i

    @property
    def _active(self) -> np.ndarray:
        return np.where(self._alive[:self._n])[0]

    def _compact(self) -> None:
        idx = np.where(self._alive[:self._n])[0]
        k = len(idx)
        self._pos[:k] = self._pos[idx]
        self._vel[:k] = self._vel[idx]
        self._sleep[:k] = self._sleep[idx]
        self._alive[:k] = True
        self._alive[k:self._n] = False
        self._n = k

    def clear_fuel(self) -> None:
        """Clear all fuel (rip)"""
        self._alive[:self._n] = False
        self._n = 0

    def spawn_fuel(self, pos: Translation3d, vel: Translation3d) -> None:
        self._add_fuel(pos.x, pos.y, pos.z, vel.x, vel.y, vel.z)

    @property
    def fuels(self) -> "FuelListView":
        """Returns proxy list of fuels"""
        return FuelListView(self._pos, self._vel, self._active)

    def spawn_starting_fuel(self, imperfect=False) -> None:
        """
        Spawn all starting fuel in the neutral zone and depots.

        :param imperfect: If True, neutral zone fuel will be 384 +- 24. Else, 384.
        ."""
        cx = _FIELD_LENGTH / 2
        cy = _FIELD_WIDTH / 2
        fuels = [
            (cx + x * (0.076 + 0.152 * j),
             cy + y * (0.0254 + 0.076 + 0.152 * i),
             _FUEL_RADIUS)
            for i in range(16 + (random.choice([-1, 0, 1]) if imperfect else 0))
            for j in range(6)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]
        self._ensure_capacity(len(fuels))
        for px, py, pz in fuels:
            self._add_fuel(px, py, pz)
        self._spawn_depot_fuel()

    def _spawn_depot_fuel(self) -> None:
        fuels = []
        for i in range(3):
            for j in range(4):
                ox = 0.076 + 0.152 * j
                fuels += [
                    (ox, 5.95 + 0.076 + 0.152 * i, _FUEL_RADIUS),
                    (ox, 5.95 - 0.076 - 0.152 * i, _FUEL_RADIUS),
                    (_FIELD_LENGTH - ox, 2.09 + 0.076 + 0.152 * i,
                     _FUEL_RADIUS),
                    (_FIELD_LENGTH - ox, 2.09 - 0.076 - 0.152 * i,
                     _FUEL_RADIUS),
                ]
        self._ensure_capacity(len(fuels))
        for px, py, pz in fuels:
            self._add_fuel(px, py, pz)

    def register_robot(
        self,
        width: meters,
        length: meters,
        bumper_height: meters,
        pose_supplier: Callable[[], Pose2d],
        field_speeds_supplier: Callable[[], ChassisSpeeds],
    ) -> None:
        """Register a robot!"""
        self.robot_pose_supplier = pose_supplier
        self.robot_speeds_supplier = field_speeds_supplier
        self.robot_width = width
        self.robot_length = length
        self.bumper_height = bumper_height
        self._broad_r_sq = (length / 2 + width / 2 + _FUEL_RADIUS) ** 2

    def register_intake(
        self,
        x_min: float, x_max: float,
        y_min: float, y_max: float,
        able_to_intake: Callable[[], bool] = lambda: True,
        callback: Callable[[], None] = lambda: None,
    ) -> None:
        """Register an intake!"""
        self.intakes.append(
            SimIntake(x_min, x_max, y_min, y_max, able_to_intake, callback)
        )

    def start(self) -> None:
        """Starts the simulation"""
        self.running = True

    def stop(self) -> None:
        """Pauses the simulation"""
        self.running = False

    def enable_air_resistance(self) -> None:
        """Enables air resistance calculations."""
        self.simulate_air_resistance = True

    def set_subticks(self, subticks: int) -> None:
        """Set amount of physics updates per tick (default 5)"""
        self.subticks = subticks

    def update_sim(self) -> None:
        """Updates the simulation. This should be run periodically."""
        if self.running:
            self.step_sim()

    def step_sim(self) -> None:
        dt = _PERIOD / self.subticks
        for _ in range(self.subticks):
            idx = self._active
            self._physics_step(idx, dt)
            self._collision_step(idx, dt)
            if self.robot_pose_supplier:
                self._handle_robot_collisions(idx)
        if self.robot_pose_supplier:
            self._handle_intakes()
        self._log()

    def launch_fuel(
        self,
        launch_velocity: meters_per_second,
        hood_angle: radians,
        turret_yaw: radians,
        launch_height: meters,
    ) -> None:
        """Launches fuel from the robot's specified arguments."""
        if (self.robot_pose_supplier is None or
                self.robot_speeds_supplier is None):
            raise RuntimeError(
                "Robot must be registered before launching fuel."
            )
        launch_pose = Pose3d(self.robot_pose_supplier()) + Transform3d(
            Translation3d(0, 0, launch_height), Rotation3d()
        )
        field_speeds = self.robot_speeds_supplier()
        horizontal_vel = math.cos(hood_angle) * launch_velocity
        vertical_vel = math.sin(hood_angle) * launch_velocity
        yaw = turret_yaw + launch_pose.rotation().z
        vx = horizontal_vel * math.cos(yaw) + field_speeds.vx
        vy = horizontal_vel * math.sin(yaw) + field_speeds.vy
        lp = launch_pose.translation()
        self._add_fuel(lp.x, lp.y, lp.z, vx, vy, vertical_vel)

    def _physics_step(self, idx, dt) -> None:
        """Updates physics."""
        if len(idx) == 0:
            return

        pos = self._pos
        vel = self._vel

        awake = idx[self._sleep[idx] < _SLEEP_THRESHOLD]
        if len(awake) == 0:
            return

        p = pos[awake]
        v = vel[awake]

        p = p + v * dt

        in_air = p[:, 2] > _FUEL_RADIUS

        if self.simulate_air_resistance:
            speed_sq = np.einsum('ij,ij->i', v, v)
            speed = np.sqrt(np.maximum(speed_sq, 1e-30))
            drag_factor = -_DRAG_OVER_MASS * speed
            ax = drag_factor * v[:, 0]
            ay = drag_factor * v[:, 1]
            az = drag_factor * v[:, 2]
            v[:, 0] += ax * dt
            v[:, 1] += ay * dt
            v[:, 2] = np.where(in_air, v[:, 2] + (_GRAVITY + az) * dt, v[:, 2])
        else:
            v[:, 2] = np.where(in_air, v[:, 2] + _GRAVITY * dt, v[:, 2])

        on_ground = (~in_air) & (p[:, 2] <= _FUEL_RADIUS + 0.03) & (
                    np.abs(v[:, 2]) < 0.05)
        v[:, 2] = np.where(on_ground, 0.0, v[:, 2])
        friction = 1.0 - _FRICTION * dt
        v[:, 0] = np.where(on_ground, v[:, 0] * friction, v[:, 0])
        v[:, 1] = np.where(on_ground, v[:, 1] * friction, v[:, 1])

        pos[awake] = p
        vel[awake] = v

        speed_sq_all = np.einsum('ij,ij->i', vel[awake], vel[awake])
        can_sleep = (speed_sq_all < _SLEEP_SPEED_SQ) & (
                    pos[awake, 2] <= _FUEL_RADIUS + 0.01)
        self._sleep[awake] = np.where(
            can_sleep,
            np.minimum(self._sleep[awake] + 1, _SLEEP_THRESHOLD),
            0,
        ).astype(np.uint8)

    def _collision_step(self, idx, dt) -> None:
        """Updates collisions."""
        if len(idx) == 0:
            return

        pos = self._pos
        vel = self._vel

        # Only update collisions for awake fuel.
        for i in idx[self._sleep[idx] < _SLEEP_THRESHOLD]:
            vx, vy = vel[i, 0], vel[i, 1]
            if vx * vx + vy * vy < 1e-12:
                continue

            _collide_edges(pos, vel, i)

            px, py = pos[i, 0], pos[i, 1]
            for li, (mn_x, mx_x, mn_y, mx_y) in enumerate(_LINE_BOUNDS):
                if (px + _FUEL_RADIUS >= mn_x and px - _FUEL_RADIUS <= mx_x and
                        py + _FUEL_RADIUS >= mn_y and py - _FUEL_RADIUS <=
                        mx_y):
                    _xz_line_collide_np(
                        pos,
                        vel,
                        i,
                        _NP_LINES[li][0],
                        _NP_LINES[li][1]
                    )

            hub = BLUE_HUB if pos[i, 0] < _FIELD_LENGTH / 2 else RED_HUB
            self._collide_hub(pos, vel, i, hub, dt)

            _collide_trench(pos, vel, i)

        self._collide_fuel_fuel(idx)

    def _collide_hub(
        self, pos: np.ndarray, vel: np.ndarray, i: int, hub: Hub, dt: float
    ) -> None:
        """Scoring, side, and net collisions for a single fuel against a
        hub."""
        pz = pos[i, 2]
        if pz > hub.ENTRY_HEIGHT + _FUEL_RADIUS:  # above hub entirely
            return
        dx = pos[i, 0] - hub.cx
        dy = pos[i, 1] - hub.cy
        if dx * dx + dy * dy > _HUB_CLOSE_SQ:
            return

        if hub.did_score(pos, vel, i, dt):
            self._sleep[i] = 0
            return

        hub.collide_side(pos, vel, i)
        hub.collide_net(pos, vel, i)

    def _collide_fuel_fuel(self, idx: np.ndarray) -> None:
        if len(idx) == 0:
            return

        grid = self._grid
        pool = self._grid_pool
        active_keys = self._grid_active_keys
        pos = self._pos

        # Return used cell lists to pool, then clear the grid
        for key in active_keys:
            lst = grid[key]
            lst.clear()
            pool.append(lst)
        active_keys.clear()
        grid.clear()

        # Build spatial grid over all active balls
        for i in idx:
            col = int(pos[i, 0] / _CELL_SIZE)
            row = int(pos[i, 1] / _CELL_SIZE)
            if 0 <= col < _GRID_COLS and 0 <= row < _GRID_ROWS:
                key = (col, row)
                if key in grid:
                    grid[key].append(i)
                else:
                    lst = pool.pop() if pool else []
                    lst.append(i)
                    grid[key] = lst
                    active_keys.append(key)

        self._current_gen += 1
        if self._current_gen == 0xFFFFFFFF: # :p
            self._pair_gen[:] = 0
            self._current_gen = 1
        gen = self._current_gen
        pair_gen = self._pair_gen

        awake = idx[self._sleep[idx] < _SLEEP_THRESHOLD]
        for i in awake:
            col = int(pos[i, 0] / _CELL_SIZE)
            row = int(pos[i, 1] / _CELL_SIZE)
            for ci in range(col - 1, col + 2):
                for ri in range(row - 1, row + 2):
                    for j in grid.get((ci, ri), []):
                        if i == j:
                            continue
                        a, b = (i, j) if i < j else (j, i)
                        if pair_gen[a, b] == gen:
                            continue
                        pair_gen[a, b] = gen
                        dx = pos[i, 0] - pos[j, 0]
                        dy = pos[i, 1] - pos[j, 1]
                        dz = pos[i, 2] - pos[j, 2]
                        if dx * dx + dy * dy + dz * dz < _FUEL_DIAM_SQ:
                            self._resolve_fuel_collision(a, b)

    def _resolve_fuel_collision(self, a: int, b: int) -> None:
        pos, vel = self._pos, self._vel
        dx = pos[a, 0] - pos[b, 0]
        dy = pos[a, 1] - pos[b, 1]
        dz = pos[a, 2] - pos[b, 2]
        dist_sq = dx * dx + dy * dy + dz * dz
        dist = math.sqrt(dist_sq) if dist_sq > 0 else 1.0
        if dist > 0:
            nx, ny, nz = dx / dist, dy / dist, dz / dist
        else:
            nx, ny, nz = 1.0, 0.0, 0.0

        impulse = 0.5 * _FUEL_COR1 * (
            (vel[b, 0] - vel[a, 0]) * nx +
            (vel[b, 1] - vel[a, 1]) * ny +
            (vel[b, 2] - vel[a, 2]) * nz
        )
        overlap = (_FUEL_DIAM - dist) * 0.5
        pos[a, 0] += nx * overlap;  pos[b, 0] -= nx * overlap
        pos[a, 1] += ny * overlap;  pos[b, 1] -= ny * overlap
        pos[a, 2] += nz * overlap;  pos[b, 2] -= nz * overlap
        vel[a, 0] += impulse * nx;  vel[b, 0] -= impulse * nx
        vel[a, 1] += impulse * ny;  vel[b, 1] -= impulse * ny
        vel[a, 2] += impulse * nz;  vel[b, 2] -= impulse * nz
        self._sleep[a] = 0
        self._sleep[b] = 0

    def _handle_robot_collisions(self, idx: np.ndarray) -> None:
        if self.robot_pose_supplier is None or self.robot_speeds_supplier is None:
            return
        robot = self.robot_pose_supplier()
        speeds = self.robot_speeds_supplier()
        rvx, rvy = speeds.vx, speeds.vy
        half_l = self.robot_length / 2
        half_w = self.robot_width / 2
        bh = self.bumper_height
        pos, vel = self._pos, self._vel

        rtx = robot.translation().x
        rty = robot.translation().y
        angle = robot.rotation().radians()
        cos_r = math.cos(angle)  # forward rotation
        sin_r = math.sin(angle)

        # cull anything outside a circle that fully contains the robot
        for i in idx:
            pz = pos[i, 2]
            if pz > bh:
                continue

            ddx = pos[i, 0] - rtx
            ddy = pos[i, 1] - rty

            # Broad phase circle check — cheap, culls most balls
            if ddx * ddx + ddy * ddy > self._broad_r_sq:
                continue

            # Transform to robot frame using transpose of rotation matrix
            rx = cos_r * ddx + sin_r * ddy
            ry = -sin_r * ddx + cos_r * ddy

            d_bot = -_FUEL_RADIUS - half_l - rx
            d_top = -_FUEL_RADIUS - half_l + rx
            d_right = -_FUEL_RADIUS - half_w - ry
            d_left = -_FUEL_RADIUS - half_w + ry

            if d_bot > 0 or d_top > 0 or d_right > 0 or d_left > 0:
                continue

            # Minimum penetration axis in robot frame
            if d_bot >= d_top and d_bot >= d_right and d_bot >= d_left:
                ox, oy = d_bot, 0.0
            elif d_top >= d_bot and d_top >= d_right and d_top >= d_left:
                ox, oy = -d_top, 0.0
            elif d_right >= d_bot and d_right >= d_top and d_right >= d_left:
                ox, oy = 0.0, d_right
            else:
                ox, oy = 0.0, -d_left

            # Rotate offset back to field frame
            off_x = cos_r * ox - sin_r * oy
            off_y = sin_r * ox + cos_r * oy

            pos[i, 0] += off_x
            pos[i, 1] += off_y

            norm = math.sqrt(off_x * off_x + off_y * off_y)
            if norm > 1e-12:
                nx_f = off_x / norm
                ny_f = off_y / norm
            else:
                nx_f, ny_f = 1.0, 0.0

            vdotn = vel[i, 0] * nx_f + vel[i, 1] * ny_f
            if vdotn < 0:
                vel[i, 0] -= vdotn * _ROBOT_COR1 * nx_f
                vel[i, 1] -= vdotn * _ROBOT_COR1 * ny_f
            rdotn = rvx * nx_f + rvy * ny_f
            if rdotn > 0:
                vel[i, 0] += rdotn * nx_f
                vel[i, 1] += rdotn * ny_f

            self._sleep[i] = 0

    def _handle_intakes(self) -> None:
        if not self.robot_pose_supplier:
            return
        robot = self.robot_pose_supplier()
        bh = self.bumper_height
        pos = self._pos

        rtx = robot.translation().x
        rty = robot.translation().y
        angle = robot.rotation().radians()
        cos_r = math.cos(angle)
        sin_r = math.sin(angle)

        removed = False
        for intake in self.intakes:
            # Hoisted outside inner loop — was previously called once per ball
            if not intake.able_to_intake():
                continue
            x_min, x_max = intake.x_min, intake.x_max
            y_min, y_max = intake.y_min, intake.y_max
            callback = intake.callback
            for i in range(self._n - 1, -1, -1):
                if not self._alive[i] or pos[i, 2] > bh:
                    continue
                ddx = pos[i, 0] - rtx
                ddy = pos[i, 1] - rty
                rx = cos_r * ddx + sin_r * ddy
                ry = -sin_r * ddx + cos_r * ddy
                if x_min <= rx <= x_max and y_min <= ry <= y_max:
                    callback()
                    self._alive[i] = False
                    removed = True
        if removed:
            self._compact()

    def _log(self) -> None:
        idx = self._active
        Logger.recordOutput(
            f"{self._table_key}/Fuel",
            [Translation3d(self._pos[i, 0], self._pos[i, 1], self._pos[i, 2]) for i in idx],
        )
        Logger.recordOutput(f"{self._table_key}/RedScore", RED_HUB.score)
        Logger.recordOutput(f"{self._table_key}/BlueScore", BLUE_HUB.score)


class FuelListView:
    """Lightweight proxy so `len(sim.fuels)` and iteration still work."""
    __slots__ = ("_pos", "_vel", "_idx")

    def __init__(self, pos: np.ndarray, vel: np.ndarray, idx: np.ndarray):
        self._pos = pos
        self._vel = vel
        self._idx = idx

    def __len__(self) -> int:
        return len(self._idx)

    def __iter__(self):
        for i in self._idx:
            yield FuelProxy(self._pos, self._vel, i)

    def __getitem__(self, n: int):
        return FuelProxy(self._pos, self._vel, int(self._idx[n]))


@dataclass
class FuelProxy:
    """Read-write proxy for a single fuel in the numpy arrays."""
    __slots__ = ("_pos", "_vel", "_i")
    _pos: np.ndarray
    _vel: np.ndarray
    _i: int

    @property
    def pos(self) -> Translation3d:
        """Position of the fuel (meters)."""
        r = self._pos[self._i]
        return Translation3d(r[0], r[1], r[2])

    @pos.setter
    def pos(self, v: Translation3d) -> None:
        self._pos[self._i] = [v.x, v.y, v.z]

    @property
    def vel(self) -> Translation3d:
        """Velocity of the fuel. (m/s)"""
        r = self._vel[self._i]
        return Translation3d(r[0], r[1], r[2])

    @vel.setter
    def vel(self, v: Translation3d) -> None:
        self._vel[self._i] = [v.x, v.y, v.z]
