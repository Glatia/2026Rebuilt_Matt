"""
Translated from https://github.com/hammerheads5000/FuelSim
v1.0.3
"""
import math
import random
from collections import defaultdict
from dataclasses import dataclass, field
from typing import ClassVar, Optional, Callable

from pykit.logger import Logger
from wpimath.geometry import (Translation3d, Translation2d, Pose2d, Pose3d,
                              Transform3d, Rotation3d, Rotation2d)
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import (seconds, kilograms_per_cubic_meter, meters,
                           kilograms, meters_per_second, radians)

### Constants
PERIOD: seconds = 0.02
GRAVITY = Translation3d(0, 0, -9.81)  # m/s^2
# Room temperature dry air density:
# https://en.wikipedia.org/wiki/Density_of_air#Dry_air
AIR_DENSITY: kilograms_per_cubic_meter = 1.2041
FIELD_COR = math.sqrt(22 / 51.5)  # Coefficient of restitution with the field
FUEL_COR = 0.5  # Coefficient of restitution with another fuel
NET_COR = 0.2  # Coefficient of restitution with the net
ROBOT_COR = 0.1  # Coefficient of restitution with a robot
FUEL_RADIUS: meters = 0.075
FIELD_LENGTH: meters = 16.51
FIELD_WIDTH: meters = 8.04
TRENCH_WIDTH: meters = 1.265
TRENCH_BLOCK_WIDTH: meters = 0.305
TRENCH_HEIGHT: meters = 0.565
TRENCH_BAR_HEIGHT: meters = 0.102
TRENCH_BAR_WIDTH: meters = 0.152
FRICTION = 0.1  # Proportion of horizontal vel to lose per sec while on ground
FUEL_MASS: kilograms = 0.448 * 0.45392
FUEL_CROSS_AREA = math.pi * FUEL_RADIUS ** 2
# Drag coefficient of smooth sphere
# https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
DRAG_COF = 0.47  # dimensionless
DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA

FIELD_XZ_LINES: tuple[tuple[Translation3d, Translation3d], ...] = (
    (Translation3d(0, 0, 0), Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0)),

    (Translation3d(3.96, 1.57, 0),
     Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165)),

    (Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165)),

    (Translation3d(4.61, 1.57, 0.165),
     Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0)),

    (Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(5.18, FIELD_WIDTH - 1.57, 0)),

    (Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
     Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165)),

    (Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
     Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165)),

    (Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
     Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0)),

    (Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
     Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0)),

    (Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT), Translation3d(
        5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT
    )),

    (Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT), Translation3d(
        5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT
    )),

    (Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
     Translation3d(
         FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT
     )),

    (Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
     Translation3d(
         FIELD_LENGTH - 3.96,
         FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
         TRENCH_HEIGHT
     )),

    (Translation3d(
        4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    ), Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        4.61 - TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH - 1.57,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    ), Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
        0,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    ), Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    )),

    (Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH - 1.57,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    ), Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
    )))


@dataclass
class Hub:
    """Handles hub interactions with fuel and collisions."""
    center: Translation2d
    exit: Translation3d
    exit_vel_x_mult: int

    _score: int = field(default=0, init=False, repr=False)

    ENTRY_HEIGHT: ClassVar[float] = 1.83
    ENTRY_RADIUS: ClassVar[float] = 0.56
    SIDE: ClassVar[float] = 1.2
    NET_HEIGHT_MAX: ClassVar[float] = 3.057
    NET_HEIGHT_MIN: ClassVar[float] = 1.5
    NET_OFFSET: ClassVar[float] = SIDE / 2 + 0.261
    NET_WIDTH: ClassVar[float] = 1.484

    def handle_hub_interaction(self, fuel: "Fuel", subticks: int) -> None:
        """Score and release fuel if needed."""
        if self.did_fuel_score(fuel, subticks):
            fuel.pos = self.exit
            fuel.vel = self.get_dispersal_velocity()
            self._score += 1

    def did_fuel_score(self, fuel: "Fuel", subticks: int) -> bool:
        """Check if fuel is withing entry bounds."""
        return fuel.pos.toTranslation2d().distance(
            self.center
        ) <= self.ENTRY_RADIUS and fuel.pos.Z() <= self.ENTRY_HEIGHT < (
                fuel.pos - (fuel.vel * (PERIOD / subticks))).Z()

    def get_dispersal_velocity(self) -> Translation3d:
        """Calculate random release velocity."""
        return Translation3d(
            self.exit_vel_x_mult * (random.random() + 0.1) * 1.5,
            random.uniform(-1, 1),
            0
        )

    @property
    def score(self) -> int:
        """Current count of fuel scored in this hub"""
        return self._score

    def reset_score(self) -> None:
        """Reset this hub's score to 0"""
        self._score = 0

    def fuel_collide_side(self, fuel: "Fuel") -> None:
        """Side rectangles for collision checks."""
        fuel_collide_rectangle(
            fuel, Translation3d(
                self.center.X() - self.SIDE / 2,
                self.center.Y() - self.SIDE / 2,
                0
            ), Translation3d(
                self.center.X() + self.SIDE / 2,
                self.center.Y() + self.SIDE / 2,
                self.ENTRY_HEIGHT - 0.1
            )
        )

    def fuel_hit_net(self, fuel: "Fuel") -> float:
        """Checks if the fuel hits the net."""
        if (
                fuel.pos.Z() > self.NET_HEIGHT_MAX or fuel.pos.Z() <
                self.NET_HEIGHT_MIN):
            return 0
        if (
                fuel.pos.Y() > self.center.Y() + self.NET_WIDTH / 2 or
                fuel.pos.Y() < self.center.Y() - self.NET_WIDTH / 2):
            return 0
        if (
                fuel.pos.X() > self.center.X() + self.NET_OFFSET *
                self.exit_vel_x_mult):
            return max(
                0,
                self.center.X() + self.NET_OFFSET * self.exit_vel_x_mult - (
                        fuel.pos.X() - FUEL_RADIUS)
            )
        else:
            return min(
                0,
                self.center.X() + self.NET_OFFSET * self.exit_vel_x_mult - (
                        fuel.pos.X() + FUEL_RADIUS)
            )


BLUE_HUB = Hub(
    Translation2d(4.61, FIELD_WIDTH / 2),
    Translation3d(5.3, FIELD_WIDTH / 2, 0.89),
    1
)
RED_HUB = Hub(
    Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
    Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
    -1
)


@dataclass
class Fuel:
    """Fuel dataclass"""
    pos: Translation3d
    vel: Translation3d = field(default_factory=Translation3d)

    def update(self, simulate_air_resistance: bool, subticks: int) -> None:
        """Update position, air resistance, and collisions."""
        self.pos += self.vel * (PERIOD / subticks)
        if self.pos.Z() > FUEL_RADIUS:
            Fg = GRAVITY * FUEL_MASS
            Fd = Translation3d()

            if simulate_air_resistance:
                speed = self.vel.norm()
                if speed > 1e-6:
                    Fd = self.vel * (-DRAG_FORCE_FACTOR * speed)

            accel = (Fg + Fd) / FUEL_MASS
            self.vel += accel * (PERIOD / subticks)
        if abs(self.vel.Z()) < 0.05 and self.pos.Z() <= FUEL_RADIUS + 0.03:
            self.vel = Translation3d(self.vel.X(), self.vel.Y(), 0)
            self.vel *= 1 - FRICTION * PERIOD / subticks
        self.handle_field_collisions(subticks)

    def handle_xz_line_collision(self,
                                 line_start: Translation3d,
                                 line_end: Translation3d
                                 ) -> None:
        """Handle a lotta collisions."""
        if self.pos.Y() < line_start.Y() or self.pos.Y() > line_end.Y():
            return
        # Convert into 2D
        start2d = Translation2d(line_start.X(), line_start.Z())
        end2d = Translation2d(line_end.X(), line_end.Z())
        pos2d = Translation2d(self.pos.X(), self.pos.Z())
        line_vec = end2d - start2d

        # Get the closest point on the line
        projected = start2d + (line_vec * (pos2d - start2d).dot(
            line_vec
        ) / line_vec.squaredNorm())

        if projected.distance(start2d) + projected.distance(
                end2d
        ) > line_vec.norm():
            return  # projected point not on the line
        dist = pos2d.distance(projected)
        if dist > FUEL_RADIUS:
            return  # not intersecting line
        # Back into 3D
        normal = Translation3d(
            -line_vec.Y(),
            0,
            line_vec.X()
        ) / line_vec.norm()

        # Apply collision response
        self.pos += normal * (FUEL_RADIUS - dist)
        if self.vel.dot(normal) > 0:
            return  # already moving away from line
        self.vel -= normal * (1 + FIELD_COR) * self.vel.dot(normal)

    def handle_field_collisions(self, subticks: int) -> None:
        """Self-explanatory."""
        # floor and bumps
        for i in range(len(FIELD_XZ_LINES)):
            self.handle_xz_line_collision(
                FIELD_XZ_LINES[i][0], FIELD_XZ_LINES[i][1]
            )

        # edges
        if self.pos.X() < FUEL_RADIUS and self.vel.X() < 0:
            self.pos += Translation3d(FUEL_RADIUS - self.pos.X(), 0, 0)
            self.vel += Translation3d(-(1 + FIELD_COR) * self.vel.X(), 0, 0)
        elif self.pos.X() > FIELD_LENGTH - FUEL_RADIUS and self.vel.X() > 0:
            self.pos += Translation3d(
                FIELD_LENGTH - FUEL_RADIUS - self.pos.X(), 0, 0
            )
            self.vel += Translation3d(-(1 + FIELD_COR) * self.vel.X(), 0, 0)

        if self.pos.Y() < FUEL_RADIUS and self.vel.Y() < 0:
            self.pos += Translation3d(0, FUEL_RADIUS - self.pos.Y(), 0)
            self.vel += Translation3d(0, -(1 + FIELD_COR) * self.vel.Y(), 0)
        elif self.pos.Y() > FIELD_WIDTH - FUEL_RADIUS and self.vel.Y() > 0:
            self.pos += Translation3d(
                0, FIELD_WIDTH - FUEL_RADIUS - self.pos.Y(), 0
            )
            self.vel += Translation3d(0, -(1 + FIELD_COR) * self.vel.Y(), 0)

        # Hubs
        self.handle_hub_collisions(BLUE_HUB, subticks)
        self.handle_hub_collisions(RED_HUB, subticks)

        self.handle_trench_collisions()

    def handle_hub_collisions(self, hub: "Hub", subticks: int) -> None:
        """Lots of collision checks..."""
        hub.handle_hub_interaction(self, subticks)
        hub.fuel_collide_side(self)

        netCollision = hub.fuel_hit_net(self)
        if netCollision != 0:
            self.pos += Translation3d(netCollision, 0, 0)
            self.vel = Translation3d(
                -self.vel.X() * NET_COR, self.vel.Y() * NET_COR, self.vel.Z()
            )

    def handle_trench_collisions(self) -> None:
        """Baseball, huh?"""
        fuel_collide_rectangle(
            self, Translation3d(3.96, TRENCH_WIDTH, 0), Translation3d(
                5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self, Translation3d(3.96, FIELD_WIDTH - 1.57, 0), Translation3d(
                5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self,
            Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
            Translation3d(
                FIELD_LENGTH - 3.96,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self,
            Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
            Translation3d(
                FIELD_LENGTH - 3.96,
                FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self,
            Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
            Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self, Translation3d(
                4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT
            ), Translation3d(
                4.61 + TRENCH_BAR_WIDTH / 2,
                FIELD_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self, Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT
            ), Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
            )
        )
        fuel_collide_rectangle(
            self, Translation3d(
                FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
                FIELD_WIDTH - 1.57,
                TRENCH_HEIGHT
            ), Translation3d(
                FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
                FIELD_WIDTH,
                TRENCH_HEIGHT + TRENCH_BAR_HEIGHT
            )
        )

    def add_impulse(self, impulse: Translation3d) -> None:
        """Update impulse for fuel velocity."""
        self.vel += impulse


def fuel_collide_rectangle(fuel: Fuel, start: Translation3d, end: Translation3d
                           ) -> None:
    """Simple rectangle collision check."""
    if (
            fuel.pos.Z() > end.Z() + FUEL_RADIUS or fuel.pos.Z() < start.Z()
            - FUEL_RADIUS):
        return  # above rectangle
    distance_to_left = start.X() - FUEL_RADIUS - fuel.pos.X()
    distance_to_right = fuel.pos.X() - end.X() - FUEL_RADIUS
    distance_to_top = fuel.pos.Y() - end.Y() - FUEL_RADIUS
    distance_to_bottom = start.Y() - FUEL_RADIUS - fuel.pos.Y()

    # not inside hub
    if (
            distance_to_left > 0 or distance_to_right > 0 or distance_to_top
            > 0
            or distance_to_bottom > 0):
        return

    # Find minimum distance to side and send corresponding collision response
    if fuel.pos.X() < start.X() or (
            distance_to_left >= distance_to_right and distance_to_left >=
            distance_to_top and distance_to_left >= distance_to_bottom):
        collision = Translation2d(distance_to_left, 0)
    elif fuel.pos.X() >= end.X() or (
            distance_to_right >= distance_to_left and distance_to_right >=
            distance_to_top and distance_to_right >= distance_to_bottom):
        collision = Translation2d(-distance_to_right, 0)
    elif fuel.pos.Y() > end.Y() or (
            distance_to_top >= distance_to_left and distance_to_top >=
            distance_to_right and distance_to_top >= distance_to_bottom):
        collision = Translation2d(0, -distance_to_top)
    else:
        collision = Translation2d(0, distance_to_bottom)

    if collision.X() != 0:
        fuel.pos += Translation3d(collision)
        fuel.vel += Translation3d(-(1 + FIELD_COR) * fuel.vel.X(), 0, 0)
    elif collision.Y() != 0:
        fuel.pos += Translation3d(collision)
        fuel.vel += Translation3d(0, -(1 + FIELD_COR) * fuel.vel.Y(), 0)


@dataclass
class SimIntake:
    """Simulated intake."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    able_to_intake: Callable[[], bool] = field(default=lambda: True)
    callback: Callable[[], None] = field(default=lambda: None)

    def shouldIntake(self,
                     fuel: Fuel,
                     robot_pose: Pose2d,
                     bumper_height: meters
                     ) -> bool:
        """Check if we're able to intake the fuel."""
        if not self.able_to_intake() or fuel.pos.Z() > bumper_height:
            return False

        fuel_relative_pos = Pose2d(
            fuel.pos.toTranslation2d(), Rotation2d()
        ).relativeTo(robot_pose).translation()

        result = (
                self.x_min <= fuel_relative_pos.X() <= self.x_max and
                self.y_min <= fuel_relative_pos.Y() <= self.y_max)
        if result:
            self.callback()
        return result


def handle_fuel_collision(a: Fuel, b: Fuel) -> None:
    """Collision."""
    normal = a.pos - b.pos
    distance = normal.norm()
    if distance == 0:
        normal = Translation3d(1, 0, 0)
        distance = 1
    normal = normal / distance
    impulse = 0.5 * (1 + FUEL_COR) * (b.vel - a.vel).dot(normal)
    intersection = FUEL_RADIUS * 2 - distance
    a.pos = a.pos + normal * (intersection / 2)
    b.pos = b.pos - normal * (intersection / 2)
    a.add_impulse(normal * impulse)
    b.add_impulse(normal * -impulse)


CELL_SIZE = 0.25
GRID_COLS = math.ceil(FIELD_LENGTH / CELL_SIZE)
GRID_ROWS = math.ceil(FIELD_WIDTH / CELL_SIZE)


class FuelSim:
    """Handles all fuel."""

    def __init__(self, table_key: str = "Fuel Simulation/") -> None:
        self._grid: dict[tuple[int, int], list[Fuel]] = defaultdict(list)
        self.fuels: list[Fuel] = []
        self.running: bool = False
        self.simulate_air_resistance: bool = False
        self.subticks: int = 5
        self.intakes: list[SimIntake] = []
        self._table_key = table_key

        self.robot_pose_supplier: Optional[Callable[[], Pose2d]] = None
        self.robot_speeds_supplier: Optional[
            Callable[[], ChassisSpeeds]] = None
        self.robot_width: float = 0
        self.robot_length: float = 0
        self.bumper_height: float = 0

    def clear_fuel(self) -> None:
        """Clears the field of fuel"""
        self.fuels.clear()

    def spawn_starting_fuel(self) -> None:
        """Spawns fuel in neutral zone and depots."""
        # Center fuel
        center = Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS)
        self.fuels += [
            Fuel(
                center + Translation3d(
                    x * (0.076 + 0.152 * j),
                    y * (0.0254 + 0.076 + 0.152 * i),
                    0
                )
            )
            for i in range(15)
            for j in range(6)
            for x, y in [(1, 1), (-1, 1), (1, -1), (-1, -1)]
        ]

        # Depots
        for i in range(3):
            for j in range(4):
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            0.076 + 0.152 * j,
                            5.95 + 0.076 + 0.152 * i,
                            FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            0.076 + 0.152 * j,
                            5.95 - 0.076 - 0.152 * i,
                            FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            FIELD_LENGTH - 0.076 - 0.152 * j,
                            2.09 + 0.076 + 0.152 * i,
                            FUEL_RADIUS
                        )
                    )
                )
                self.fuels.append(
                    Fuel(
                        Translation3d(
                            FIELD_LENGTH - 0.076 - 0.152 * j,
                            2.09 - 0.076 - 0.152 * i,
                            FUEL_RADIUS
                        )
                    )
                )

    def log_fuels(self) -> None:
        """Adds array of `Translation3d`'s to NetworkTables at tableKey +
        "/Fuels"""
        Logger.recordOutput(
            f"{self._table_key}/Fuel", [fuel.pos for fuel in self.fuels]
        )

    def start(self) -> None:
        """Start the simulation. `updateSim` must still be called every loop"""
        self.running = True

    def stop(self) -> None:
        """Pause the simulation."""
        self.running = False

    def enable_air_resistance(self) -> None:
        """Enables accounting for drag force in physics step"""
        self.simulate_air_resistance = True

    def set_subticks(self, subticks: int) -> None:
        """Sets the number of physics iterations per loop (0.02s)"""
        self.subticks = subticks

    def register_robot(self,
                       width: meters,
                       length: meters,
                       bumper_height: meters,
                       pose_supplier: Callable[[], Pose2d],
                       field_speeds_supplier: Callable[[], ChassisSpeeds]
                       ) -> None:
        """Registers a robot with the fuel simulator"""
        self.robot_pose_supplier = pose_supplier
        self.robot_speeds_supplier = field_speeds_supplier
        self.robot_width = width
        self.robot_length = length
        self.bumper_height = bumper_height

    def update_sim(self) -> None:
        """
        To be called periodically
        Will do nothing if sim is not running
        """
        if self.running:
            self.step_sim()

    def step_sim(self) -> None:
        """Run the simulation forward 1 time step (0.02s)"""
        for _ in range(self.subticks):
            for fuel in self.fuels:
                fuel.update(self.simulate_air_resistance, self.subticks)

            self.handle_fuel_collisions(self.fuels)

            if self.robot_pose_supplier is not None:
                self.handle_robot_collisions(self.fuels)
                self.handle_intakes(self.fuels)

        self.log_fuels()

    def spawn_fuel(self, pos: Translation3d, vel: Translation3d) -> None:
        """Adds a fuel onto the field"""
        self.fuels.append(Fuel(pos, vel))

    def launch_fuel(self,
                    launch_velocity: meters_per_second,
                    hood_angle: radians,
                    turret_yaw: radians,
                    launch_height: meters
                    ) -> None:
        if (
                self.robot_pose_supplier is None or
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

        yaw = turret_yaw + launch_pose.rotation().Z()
        x_vel = horizontal_vel * math.cos(yaw)
        y_vel = horizontal_vel * math.sin(yaw)
        x_vel += field_speeds.vx
        y_vel += field_speeds.vy

        self.spawn_fuel(
            launch_pose.translation(),
            Translation3d(x_vel, y_vel, vertical_vel)
        )

    def handle_robot_collision(self,
                               fuel: Fuel,
                               robot: Pose2d,
                               robot_vel: Translation2d
                               ) -> None:
        """Handle a single robot to fuel collision."""
        relative_pos = Pose2d(
            fuel.pos.toTranslation2d(), Rotation2d()
        ).relativeTo(robot).translation()

        if fuel.pos.Z() > self.bumper_height:
            return  # above bumpers
        distance_to_bottom = (
                -FUEL_RADIUS - self.robot_length / 2 - relative_pos.X())
        distance_to_top = (-FUEL_RADIUS - self.robot_length / 2 +
                           relative_pos.X())
        distance_to_right = (-FUEL_RADIUS - self.robot_width / 2 -
                             relative_pos.Y())
        distance_to_left = (-FUEL_RADIUS - self.robot_width / 2 +
                            relative_pos.Y())

        # not inside robot
        if (
                distance_to_bottom > 0 or distance_to_top > 0 or
                distance_to_right
                > 0 or distance_to_left > 0):
            return

        # Find minimum distance to side and send corresponding collision
        # response
        if (
                distance_to_bottom >= distance_to_top and distance_to_bottom >=
                distance_to_right and distance_to_bottom >= distance_to_left):
            pos_offset = Translation2d(distance_to_bottom, 0)
        elif (
                distance_to_top >= distance_to_bottom and distance_to_top >=
                distance_to_right and distance_to_top >= distance_to_left):
            pos_offset = Translation2d(-distance_to_top, 0)
        elif (
                distance_to_right >= distance_to_bottom and
                distance_to_right >=
                distance_to_top and distance_to_right >= distance_to_left):
            pos_offset = Translation2d(0, distance_to_right)
        else:
            pos_offset = Translation2d(0, -distance_to_left)

        pos_offset = pos_offset.rotateBy(robot.rotation())
        fuel.pos += Translation3d(pos_offset)
        normal = pos_offset / pos_offset.norm()
        if fuel.vel.toTranslation2d().dot(normal) < 0:
            fuel.add_impulse(
                Translation3d(
                    normal * (-fuel.vel.toTranslation2d().dot(normal) * (
                            1 + ROBOT_COR))
                )
            )
        if robot_vel.dot(normal) > 0:
            fuel.add_impulse(Translation3d(normal * robot_vel.dot(normal)))

    def handle_robot_collisions(self, fuels: list[Fuel]) -> None:
        """Plural."""
        robot = self.robot_pose_supplier()
        speeds = self.robot_speeds_supplier()
        robot_vel = Translation2d(speeds.vx, speeds.vy)

        for fuel in fuels:
            self.handle_robot_collision(fuel, robot, robot_vel)

    def handle_intakes(self, fuels: list[Fuel]) -> None:
        """Update intakes."""
        robot = self.robot_pose_supplier()
        for intake in self.intakes:
            for i in reversed(range(len(fuels))):
                if intake.shouldIntake(fuels[i], robot, self.bumper_height):
                    fuels.pop(i)

    def handle_fuel_collisions(self, fuels: list[Fuel]) -> None:
        """There is so many freaking collision calls."""
        # Clear grid
        self._grid.clear()

        # Populate grid
        for fuel in fuels:
            col = int(fuel.pos.X() / CELL_SIZE)
            row = int(fuel.pos.Y() / CELL_SIZE)

            if 0 <= col < GRID_COLS and 0 <= row < GRID_ROWS:
                self._grid[col, row].append(fuel)

        # Check collisions
        for fuel in fuels:
            col = int(fuel.pos.X() / CELL_SIZE)
            row = int(fuel.pos.Y() / CELL_SIZE)

            for i in range(col - 1, col + 2):
                for j in range(row - 1, row + 2):
                    if 0 <= i < GRID_COLS and 0 <= j < GRID_ROWS:
                        for other in self._grid.get((i, j), []):
                            if (fuel is not other and fuel.pos.distance(
                                    other.pos
                            ) < FUEL_RADIUS * 2 and id(fuel) < id(other)):
                                handle_fuel_collision(fuel, other)

    def register_intake(self,
                        x_min: float,
                        x_max: float,
                        y_min: float,
                        y_max: float,
                        able_to_intake: Callable[[], bool] = lambda: True,
                        callback: Callable[[], None] = lambda: None
                        ) -> None:
        """Register an intake."""
        self.intakes.append(
            SimIntake(x_min, x_max, y_min, y_max, able_to_intake, callback)
        )
