"""
Hood subsystem

TO DO:
Integrate positions for passing (may switch to state subsystem for this)
"""
from math import atan, sqrt, degrees
from typing import Callable

from pathplannerlib.auto import FlippingUtil, AutoBuilder
from pykit.logger import Logger
from wpilib import Alert, DriverStation
from wpimath.filter import Debouncer
from wpimath.geometry import Pose2d, Rotation2d, Pose3d

from constants import Constants
from subsystems import Subsystem
from subsystems.hood.io import HoodIO


# pylint: disable=too-many-instance-attributes
class HoodSubsystem(Subsystem):
    """Subsystem for hood."""

    def __init__(self, io: HoodIO, robot_pose_supplier: Callable[[], Pose2d]) -> None:
        super().__init__()

        self.io = io
        self.alliance = DriverStation.getAlliance()

        self.robot_pose_supplier = robot_pose_supplier

        self.inputs = HoodIO.HoodIOInputs()
        self.hood_disconnected_alert = Alert("Hood motor is disconnected.", Alert.AlertType.kError)

        self.at_set_point_debounce = Debouncer(0.1, Debouncer.DebounceType.kRising)

        self.hub_pose = Constants.FieldConstants.HUB_POSE  # blue hub
        self.launch_speed =  10.03 # meters per second
        self.distance = 1.0000000
        self.angle = 1.0000000

    def update_angle(self) -> None:
        """Updates hood angle."""
        self.angle = degrees(atan(  # calculates angle
            (self.launch_speed ** 2 +
             sqrt(
                 abs(self.launch_speed ** 4 -
                 9.80665 *
                 (9.80665 * self.distance ** 2 +
                  3 * Constants.FieldConstants.HUB_HEIGHT * self.launch_speed ** 2))))
            / (9.80665 * self.distance)))


    def periodic(self) -> None:
        """Runs stuff periodically (every 20 ms)."""
        self.io.update_inputs(self.inputs)
        Logger.processInputs("Hood", self.inputs)
        Logger.recordOutput("Hood/Calculated Angle", self.angle)
        Logger.recordOutput("Hood/Distance", self.distance)

        self.distance = (self.robot_pose_supplier()
                         .translation().distance(self.hub_pose.translation()))

        self.io.set_position(Rotation2d.fromDegrees(self.angle)) # convert degrees to rotations

        self.hood_disconnected_alert.set(not self.inputs.hood_connected)

        self.update_angle()

        self.hub_pose = Constants.FieldConstants.HUB_POSE if not (
            AutoBuilder.shouldFlip()) else FlippingUtil.flipFieldPose(
            Constants.FieldConstants.HUB_POSE)


    def get_component_pose(self) -> Pose3d:
        """For advantage scope modelling (placeholder)."""
