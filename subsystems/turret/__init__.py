from enum import auto, Enum
from pykit.logger import Logger
from wpilib import Alert
from typing import Final, Callable
from constants import Constants
from subsystems import Subsystem
from subsystems.turret.io import TurretIO
from math import atan2, pi
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import rotationsToRadians
from wpilib import DriverStation
from subsystems import StateSubsystem

class TurretSubsystem(StateSubsystem):

    class SubsystemState(Enum):
        MANUAL = auto()
        HUB = auto()
        DEPOT = auto()
        OUTPOST = auto()

    _state_configs: dict[SubsystemState, bool] = {
        SubsystemState.HUB: True,
        SubsystemState.DEPOT: True,
        SubsystemState.OUTPOST: True,
        SubsystemState.MANUAL: False

    }

    def __init__(self, io: TurretIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__("Turret", self.SubsystemState.MANUAL)

        self._io: Final[TurretIO] = io
        self._inputs = TurretIO.TurretIOInputs()
        self.set_desired_state(TurretSubsystem.SubsystemState.MANUAL)
        self.robot_pose_supplier = robot_pose_supplier

        self.turret_disconnected_alert = Alert("Turret motor is disconnected.", Alert.AlertType.kError)

        self.independent_rotation = Rotation2d(0)
        self.current_radians = 0.0
        self.target_radians = 0.0

        self.x = 6.7
        self.y = 4.1

    def periodic(self):

        # Update inputs from hardware/simulation
        self._io.update_inputs(self._inputs)

        # Log inputs to PyKit
        Logger.processInputs("Turret", self._inputs)
        Logger.recordOutput("Turret/X Distance", self.x)
        Logger.recordOutput("Turret/Y Distance", self.y)
        Logger.recordOutput("Turret/Robot Current Radians", self.current_radians)
        Logger.recordOutput("Turret/Target Radians", self.target_radians)

        # Update alerts
        self.turret_disconnected_alert.set(not self._inputs.turret_connected)

        self.current_radians = self.robot_pose_supplier().rotation().radians() + self.independent_rotation.radians()

        if self.get_current_state() != self.SubsystemState.MANUAL:
            self.rotate_to_goal(self.get_current_state())
        
    def get_radians_to_goal(self) -> float:
        """
        Field-frame angle (radians) from robot to goal. 0 = +X (red alliance wall), CCW positive.
        Returns 0 for MANUAL or if robot is at goal.
        """
        state = self.get_current_state()
        if state == self.SubsystemState.MANUAL:
            return 0.0

        robot = self.robot_pose_supplier().translation()
        goal = self._goal_pose_for_state(state).translation()

        dx = goal.X() - robot.X()
        dy = goal.Y() - robot.Y()
        self.x = abs(dx)
        self.y = abs(dy)

        if dx == 0.0 and dy == 0.0:
            return 0.0
        return atan2(dy, dx)

    def _goal_pose_for_state(self, state: SubsystemState) -> Pose2d:
        """Goal pose for the given state and current alliance."""
        is_blue = DriverStation.getAlliance() == DriverStation.Alliance.kBlue
        match state:
            case self.SubsystemState.HUB:
                return Constants.GoalLocations.BLUE_HUB if is_blue else Constants.GoalLocations.RED_HUB
            case self.SubsystemState.OUTPOST:
                return Constants.GoalLocations.BLUE_OUTPOST_PASS if is_blue else Constants.GoalLocations.RED_OUTPOST_PASS
            case self.SubsystemState.DEPOT:
                return Constants.GoalLocations.BLUE_DEPOT_PASS if is_blue else Constants.GoalLocations.RED_DEPOT_PASS
            case _:
                return Constants.GoalLocations.BLUE_HUB  # fallback, caller should not use for MANUAL

    def rotate_to_goal(self, target: SubsystemState):
        """Aim turret at goal. Commands absolute turret angle in robot frame (shortest path)."""
        self.set_desired_state(target)
        if self.get_current_state() == self.SubsystemState.MANUAL:
            return

        # Field angle to goal (direction from robot to goal in field coords, CCW positive)
        field_angle_to_goal = self.get_radians_to_goal()
        robot_rotation = self.robot_pose_supplier().rotation().radians()

        # Turret angle relative to robot forward. Turret motor is CLOCKWISE_POSITIVE, field is CCW positive,
        # so negate so that "turn toward goal" matches physical turret direction.
        desired_turret = -(field_angle_to_goal - robot_rotation)
        # Normalize desired to [-pi, pi]
        while desired_turret > pi:
            desired_turret -= 2 * pi
        while desired_turret < -pi:
            desired_turret += 2 * pi

        # Shortest path: command the equivalent angle closest to current position to avoid full revolution
        current_turret = rotationsToRadians(
            self._inputs.turret_position - self._inputs.turret_zero_position
        )
        delta = desired_turret - current_turret
        while delta > pi:
            delta -= 2 * pi
        while delta < -pi:
            delta += 2 * pi
        command_turret = current_turret + delta

        self.target_radians = field_angle_to_goal
        self._io.set_position(command_turret)

    def rotate_manually(self, axis: float): # Axis is the value of the X-axis from a joystick
        target_velocity = axis * Constants.TurretConstants.MAX_MANUAL_VELOCITY
        self._io.set_velocity(target_velocity)

    def get_current_state(self) -> SubsystemState | None:
        """get state"""
        return super().get_current_state()

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """set state"""
        if not super().set_desired_state(desired_state):
            return

        auto_aim = self._state_configs.get(desired_state, False)

        if auto_aim:
            self.rotate_to_goal(desired_state)
        else:
            self.rotate_manually(0.0)
