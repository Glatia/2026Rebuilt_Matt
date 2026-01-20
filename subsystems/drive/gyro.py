from abc import ABC
from copy import deepcopy
from dataclasses import dataclass, field
from typing import List, Final

from phoenix6 import StatusSignal, BaseStatusSignal
from phoenix6.configs import Pigeon2Configuration
from phoenix6.hardware import Pigeon2
from pykit.autolog import autolog
from wpimath.geometry import Rotation2d
from wpimath.units import *

from subsystems.toast import odomFrequency, gyroId
from util import tryUntilOk, PhoenixOdometryThread, LoggedTunableNumber


class GyroIO(ABC):

    @autolog
    @dataclass
    class GyroIOInputs:
        connected: bool = False

        yawPosition: Rotation2d = field(default_factory=Rotation2d)
        yawVelocityRadPerSec: float = 0.0

        pitchPosition: Rotation2d = field(default_factory=Rotation2d)
        pitchVelocityRadPerSec: float = 0.0

        rollPosition: Rotation2d = field(default_factory=Rotation2d)
        rollVelocityRadPerSec: float = 0.0

        odometryYawTimestamps: List[float] = field(default_factory=list)
        odometryYawPositions: List[Rotation2d] = field(default_factory=list)

    def updateInputs(self, inputs: GyroIOInputs) -> None:
        pass

class GyroIOPigeon2(GyroIO):

    def __init__(self) -> None:
        self._pigeon: Final[Pigeon2] = Pigeon2(gyroId, "*")
        self._yaw: Final[StatusSignal[degrees]] = self._pigeon.get_yaw()
        self._yawVelocity: Final[StatusSignal[degrees_per_second]] = self._pigeon.get_angular_velocity_z_world()
        self._pitch: Final[StatusSignal[degrees]] = self._pigeon.get_pitch()
        self._pitchVelocity: Final[StatusSignal[degrees_per_second]] = self._pigeon.get_angular_velocity_x_world()
        self._roll: Final[StatusSignal[degrees]] = self._pigeon.get_roll()
        self._rollVelocity: Final[StatusSignal[degrees_per_second]] = self._pigeon.get_angular_velocity_y_world()
        self._pigeon.configurator.apply(Pigeon2Configuration())
        self._pigeon.configurator.set_yaw(0)
        self._yaw.set_update_frequency(odomFrequency)
        BaseStatusSignal.set_update_frequency_for_all(
            50, self._pitch, self._roll, self._yawVelocity, self._pitchVelocity, self._rollVelocity
        )
        self._pigeon.optimize_bus_utilization()
        self._yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        self._yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(deepcopy(self._yaw))
        tryUntilOk(5, lambda: self._pigeon.set_yaw(0, 0.25))

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = BaseStatusSignal.refresh_all(self._yaw, self._yawVelocity).is_ok()
        inputs.yawPosition = Rotation2d.fromDegrees(self._yaw.value_as_double)
        inputs.yawVelocityRadPerSec = degreesToRadians(self._yawVelocity.value_as_double)
        inputs.pitchPosition = Rotation2d.fromDegrees(self._pitch.value_as_double)
        inputs.pitchVelocityRadPerSec = degreesToRadians(self._pitchVelocity.value_as_double)
        inputs.rollPosition = Rotation2d.fromDegrees(self._roll.value_as_double)
        inputs.rollVelocityRadPerSec = degreesToRadians(self._rollVelocity.value_as_double)

        inputs.odometryYawTimestamps = [float(ts) for ts in self._yawTimestampQueue]
        inputs.odometryYawPositions = [Rotation2d.fromDegrees(pos) for pos in self._yawPositionQueue]
        self._yawTimestampQueue.clear()
        self._yawPositionQueue.clear()

class GyroIOSim(GyroIO):

    _simulatedGyroPitch: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/SimulatedGyroPitch", 0.0)
    _simulatedGyroRoll: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/SimulatedGyroRoll", 0.0)

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = False

        inputs.pitchPosition = Rotation2d.fromDegrees(self._simulatedGyroPitch.get())
        inputs.pitchVelocityRadPerSec = 0.0
        inputs.rollPosition = Rotation2d.fromDegrees(self._simulatedGyroRoll.get())
        inputs.rollVelocityRadPerSec = 0.0
