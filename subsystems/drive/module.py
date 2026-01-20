from abc import ABC
from collections import deque
from concurrent.futures.thread import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import field
from threading import Lock

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut, PositionVoltage, VelocityVoltage, TorqueCurrentFOC, PositionTorqueCurrentFOC, VelocityTorqueCurrentFOC
from phoenix6.hardware import TalonFX, CANcoder, ParentDevice
from phoenix6.signals import NeutralModeValue, InvertedValue, FeedbackSensorSourceValue
from phoenix6.swerve import SteerFeedbackType
from pykit.autolog import autolog
from pykit.logger import Logger
from wpilib import Alert, Timer, RobotBase
from wpilib.simulation import DCMotorSim
from wpimath.controller import PIDController, SimpleMotorFeedforwardMeters
from wpimath.filter import Debouncer
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.system.plant import LinearSystemId

from constants import Constants
from subsystems.toast import *
from util import tryUntilOk, LoggedTunableNumber, PhoenixOdometryThread

rotations_per_second = float


class ModuleIO(ABC):

    @autolog
    @dataclass
    class ModuleIOInputs:
        driveConnected: bool = False
        drivePosition: radians = 0.0
        driveVelocity: radians_per_second = 0.0
        driveAppliedVolts: volts = 0.0
        driveCurrent: amperes = 0.0
        driveTemperature: celsius = 0.0

        steerConnected: bool = False
        steerEncoderConnected: bool = False
        steerAbsolutePosition: Rotation2d = field(default_factory=Rotation2d)
        steerPosition: Rotation2d = field(default_factory=Rotation2d)
        steerVelocity: radians_per_second = 0.0
        steerAppliedVolts: volts = 0.0
        steerCurrentAmps: amperes = 0.0
        steerTemperature: celsius = 0.0

        odometryTimestamps: List[float] = field(default_factory=list)
        odometryDrivePositions: List[radians] = field(default_factory=list)
        odometrySteerPositions: List[Rotation2d] = field(default_factory=Rotation2d)

    def updateInputs(self, inputs: ModuleIOInputs) -> None:
        pass

    def setDriveOpenLoop(self, output: float) -> None:
        pass

    def setSteerOpenLoop(self, output: float) -> None:
        pass

    def setDriveVelocity(self, velocity: radians_per_second, feedforward: float) -> None:
        pass

    def setSteerPosition(self, rotation: Rotation2d) -> None:
        pass

    def setDrivePID(self, kP: float, kI: float, kD: float) -> None:
        pass

    def setSteerPID(self, kP: float, kI: float, kD: float) -> None:
        pass

    def setBrakeMode(self, enabled: bool) -> None:
        pass

class ModuleIOTalonFX(ModuleIO):
    _steerFeedbackType: Final[SteerFeedbackType] = SteerFeedbackType.FUSED_CANCODER

    _brakeModeExecutor: Final[ThreadPoolExecutor] = ThreadPoolExecutor(max_workers=8)

    _driveConnectedDebounce: Final[Debouncer] = Debouncer(0.5, Debouncer.DebounceType.kFalling)
    _steerConnectedDebounce: Final[Debouncer] = Debouncer(0.5, Debouncer.DebounceType.kFalling)
    _steerEncoderConnectedDebounce: Final[Debouncer] = Debouncer(0.5, Debouncer.DebounceType.kFalling)

    def __init__(self, config: ModuleConfig):
        self._driveTalon = TalonFX(config.driveMotorId, "*")
        self._steerTalon = TalonFX(config.steerMotorId, "*")
        self._cancoder = CANcoder(config.encoderId, "*")

        # Configure drive motor
        self._driveConfig = TalonFXConfiguration()
        self._driveConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._driveConfig.feedback.sensor_to_mechanism_ratio = driveRatio
        self._driveConfig.torque_current.peak_forward_torque_current = driveStatorLimit
        self._driveConfig.torque_current.peak_reverse_torque_current = -driveStatorLimit
        self._driveConfig.current_limits.stator_current_limit = driveStatorLimit
        self._driveConfig.current_limits.stator_current_limit_enable = True
        self._driveConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if config.isDriveInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        tryUntilOk(5, lambda: self._driveTalon.configurator.apply(self._driveConfig, 0.25))
        tryUntilOk(5, lambda: self._driveTalon.set_position(0, 0.25))

        # Configure steer motor
        self._steerConfig = TalonFXConfiguration()
        self._steerConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        self._steerConfig.feedback.feedback_remote_sensor_id = config.encoderId
        match self._steerFeedbackType:
            case SteerFeedbackType.REMOTE_CANCODER:
                self._steerConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER
            case SteerFeedbackType.FUSED_CANCODER:
                self._steerConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.REMOTE_CANCODER
            case SteerFeedbackType.SYNC_CANCODER:
                self._steerConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.SYNC_CANCODER
            case _:
                raise RuntimeError("FeedbackSource not supported.")
        self._steerConfig.feedback.rotor_to_sensor_ratio = steerRatio
        self._steerConfig.closed_loop_general.continuous_wrap = True
        self._steerConfig.torque_current.peak_forward_torque_current = steerStatorLimit
        self._steerConfig.torque_current.peak_reverse_torque_current = -steerStatorLimit
        self._steerConfig.current_limits.stator_current_limit = steerStatorLimit
        self._steerConfig.current_limits.stator_current_limit_enable = True
        self._steerConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if config.isSteerInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self._steerConfig.motion_magic.motion_magic_cruise_velocity = 100 / steerRatio
        self._steerConfig.motion_magic.motion_magic_acceleration = self._steerConfig.motion_magic.motion_magic_cruise_velocity / 0.1
        self._steerConfig.motion_magic.motion_magic_expo_k_v = 0.12 * steerRatio
        self._steerConfig.motion_magic.motion_magic_expo_k_a = 0.1
        tryUntilOk(5, lambda: self._steerTalon.configurator.apply(self._steerConfig, 0.25))

        # Create timestamp queue
        self._timestampQueue = deque()

        # Create locks
        self._driveLock = Lock()
        self._steerLock = Lock()

        # Create drive status signals
        self._drivePosition = self._driveTalon.get_position()
        self._drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(deepcopy(self._drivePosition))
        self._driveVelocity = self._driveTalon.get_velocity()
        self._driveAppliedVolts = self._driveTalon.get_motor_voltage()
        self._driveCurrent = self._driveTalon.get_stator_current()
        self._driveTemperature = self._driveTalon.get_device_temp()

        # Create steer status signals
        self._steerAbsolutePosition = self._cancoder.get_absolute_position()
        self._steerPosition = self._steerTalon.get_position()
        self._steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(deepcopy(self._steerPosition))
        self._steerVelocity = self._steerTalon.get_velocity()
        self._steerAppliedVolts = self._steerTalon.get_motor_voltage()
        self._steerCurrent = self._steerTalon.get_stator_current()
        self._steerTemperature = self._steerTalon.get_device_temp()

        # Configure periodic frames
        BaseStatusSignal.set_update_frequency_for_all(odomFrequency, self._drivePosition, self._steerPosition)
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._driveVelocity,
            self._driveAppliedVolts,
            self._driveCurrent,
            self._driveTemperature,
            self._steerAbsolutePosition,
            self._steerVelocity,
            self._steerAppliedVolts,
            self._steerCurrent,
            self._steerTemperature
        )
        ParentDevice.optimize_bus_utilization_for_all(self._driveTalon, self._steerTalon)

        # Voltage control requests
        self._voltageRequest = VoltageOut(0)
        self._positionVoltageRequest = PositionVoltage(0)
        self._velocityVoltageRequest = VelocityVoltage(0)

        # Torque-current control requests
        self._torqueCurrentRequest = TorqueCurrentFOC(0)
        self._positionTorqueCurrentRequest = PositionTorqueCurrentFOC(0)
        self._velocityTorqueCurrentRequest = VelocityTorqueCurrentFOC(0)

    def updateInputs(self, inputs: ModuleIO.ModuleIOInputs) -> None:
        # Refresh all signals
        driveStatus = BaseStatusSignal.refresh_all(self._drivePosition, self._driveVelocity, self._driveAppliedVolts, self._driveCurrent, self._driveTemperature)
        steerStatus = BaseStatusSignal.refresh_all(self._steerPosition, self._steerVelocity, self._steerAppliedVolts, self._steerCurrent, self._steerTemperature)
        steerEncoderStatus = BaseStatusSignal.refresh_all(self._steerAbsolutePosition)

        # Update positions (with latency compensation)
        drive_rot = BaseStatusSignal.get_latency_compensated_value(self._drivePosition, self._driveVelocity)
        steer_rot = BaseStatusSignal.get_latency_compensated_value(self._steerPosition, self._steerVelocity)

        # Due to coupling with the azimuth, back out drive_rot by azimuth coupling.
        drive_rot -= steer_rot * couplingRatio

        # Update drive inputs
        inputs.driveConnected = self._driveConnectedDebounce.calculate(driveStatus.is_ok())
        inputs.drivePosition = rotationsToRadians(drive_rot)
        inputs.driveVelocity = rotationsToRadians(self._driveVelocity.value_as_double)
        inputs.driveAppliedVolts = self._driveAppliedVolts.value_as_double
        inputs.driveCurrent = self._driveCurrent.value_as_double
        inputs.driveTemperature = self._driveTemperature.value_as_double

        # Update steer inputs
        inputs.steerConnected = self._steerConnectedDebounce.calculate(steerStatus.is_ok())
        inputs.steerEncoderConnected = self._steerEncoderConnectedDebounce.calculate(steerEncoderStatus.is_ok())
        inputs.steerAbsolutePosition = Rotation2d.fromRotations(self._steerAbsolutePosition.value_as_double)
        inputs.steerPosition = Rotation2d.fromRotations(steer_rot)
        inputs.steerVelocity = rotationsToRadians(self._steerVelocity.value_as_double)
        inputs.steerAppliedVolts = self._steerAppliedVolts.value_as_double
        inputs.steerCurrentAmps = self._steerCurrent.value_as_double
        inputs.steerTemperature = self._steerTemperature.value_as_double

        # Update odometry inputs
        inputs.odometryTimestamps = [float(value) for value in self._timestampQueue]
        inputs.odometryDrivePositions = [rotationsToRadians(pos) for pos in self._drivePositionQueue]
        inputs.odometrySteerPositions = [Rotation2d.fromRotations(pos) for pos in self._steerPositionQueue]
        self._timestampQueue.clear()
        self._drivePositionQueue.clear()
        self._steerPositionQueue.clear()

    def setDriveOpenLoop(self, output: float) -> None:
        match driveClosedLoopOutput:
            case ClosedLoopOutputType.VOLTAGE:
                self._driveTalon.set_control(self._voltageRequest.with_output(output))
            case ClosedLoopOutputType.TORQUE_CURRENT_FOC:
                self._driveTalon.set_control(self._torqueCurrentRequest.with_output(output))

    def setSteerOpenLoop(self, output: float) -> None:
        match steerClosedLoopOutput:
            case ClosedLoopOutputType.VOLTAGE:
                self._steerTalon.set_control(self._voltageRequest.with_output(output))
            case ClosedLoopOutputType.TORQUE_CURRENT_FOC:
                self._steerTalon.set_control(self._torqueCurrentRequest.with_output(output))
    
    def setDriveVelocity(self, velocity: radians_per_second, feedforward: float) -> None:
        velocityRotPerSec = radiansToRotations(velocity)
        match driveClosedLoopOutput:
            case ClosedLoopOutputType.VOLTAGE:
                self._driveTalon.set_control(self._velocityVoltageRequest.with_velocity(velocityRotPerSec).with_feed_forward(feedforward))
            case ClosedLoopOutputType.TORQUE_CURRENT_FOC:
                self._driveTalon.set_control(self._velocityTorqueCurrentRequest.with_velocity(velocityRotPerSec).with_feed_forward(feedforward))
                
    def setSteerPosition(self, rotation: Rotation2d) -> None:
        match steerClosedLoopOutput:
            case ClosedLoopOutputType.VOLTAGE:
                self._steerTalon.set_control(self._positionVoltageRequest.with_position(rotation.degrees() / 360))
            case ClosedLoopOutputType.TORQUE_CURRENT_FOC:
                self._steerTalon.set_control(self._positionTorqueCurrentRequest.with_position(rotation.degrees() / 360))
                
    def setDrivePID(self, kP: float, kI: float, kD: float) -> None:
        self._driveConfig.slot0.k_p = kP
        self._driveConfig.slot0.k_i = kI
        self._driveConfig.slot0.k_d = kD
        tryUntilOk(5, lambda: self._driveTalon.configurator.apply(self._driveConfig, 0.25))

    def setSteerPID(self, kP: float, kI: float, kD: float) -> None:
        self._steerConfig.slot0.k_p = kP
        self._steerConfig.slot0.k_i = kI
        self._steerConfig.slot0.k_d = kD
        tryUntilOk(5, lambda: self._steerTalon.configurator.apply(self._steerConfig, 0.25))

    def setBrakeMode(self, enabled: bool) -> None:
        mode = NeutralModeValue.BRAKE if enabled else NeutralModeValue.COAST

        def _apply(lock: Lock, config: TalonFXConfiguration, talon: TalonFX, neutralMode: NeutralModeValue):
            with lock:
                config.motor_output.neutral_mode = neutralMode
                tryUntilOk(5, lambda: talon.configurator.apply(config, 0.25))

        self._brakeModeExecutor.submit(
            lambda: _apply(self._driveLock, self._driveConfig, self._driveTalon, mode)
        )
        self._brakeModeExecutor.submit(
            lambda: _apply(self._steerLock, self._steerConfig, self._steerTalon, mode)
        )


class ModuleIOSim(ModuleIO):

    def __init__(self, config: ModuleConfig) -> None:
        # Create drive and steer sim models
        self._driveSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(driveMotor, 0.01, driveRatio),
            driveMotor
        )
        self._steerSim = DCMotorSim(
            LinearSystemId.DCMotorSystem(steerMotor, 0.1, steerRatio),
            steerMotor
        )

        self._driveClosedLoop = False
        self._steerClosedLoop = False
        self._driveController = PIDController(0.05, 0, 0)
        self._steerController = PIDController(8, 0, 0)

        self._driveFFVolts = 0.0
        self._driveAppliedVolts = 0.0
        self._steerAppliedVolts = 0.0

        # Enable wrapping for steer PID
        self._steerController.enableContinuousInput(-math.pi, math.pi)

    def updateInputs(self, inputs: ModuleIO.ModuleIOInputs) -> None:
        # Run closed-loop control
        if self._driveClosedLoop:
            self._driveAppliedVolts = self._driveFFVolts + self._driveController.calculate(self._driveSim.getAngularVelocity())
        else:
            self._driveController.reset()

        if self._steerClosedLoop:
            self._steerAppliedVolts = self._steerController.calculate(self._steerSim.getAngularPosition())
        else:
            self._steerController.reset()

        # Update simulation state
        self._driveSim.setInputVoltage(max(-12, min(self._driveAppliedVolts, 12)))
        self._steerSim.setInputVoltage(max(-12, min(self._steerAppliedVolts, 12)))
        self._driveSim.update(0.02)
        self._steerSim.update(0.02)

        # Update drive inputs
        inputs.driveConnected = True
        inputs.drivePosition = self._driveSim.getAngularPosition()
        inputs.driveVelocity = self._driveSim.getAngularVelocity()
        inputs.driveAppliedVolts = self._driveAppliedVolts
        inputs.driveCurrent = abs(self._driveSim.getCurrentDraw())
        inputs.driveTemperature = 15715.0 / 900.0

        # Update turn inputs
        inputs.steerConnected = True
        inputs.steerEncoderConnected = True
        inputs.steerAbsolutePosition = Rotation2d(self._steerSim.getAngularPosition())
        inputs.steerPosition = Rotation2d(self._steerSim.getAngularPosition())
        inputs.steerVelocity = self._steerSim.getAngularVelocity()
        inputs.steerAppliedVolts = self._steerAppliedVolts
        inputs.steerCurrentAmps = abs(self._steerSim.getCurrentDraw())
        inputs.steerTemperature = 15715.0 / 900.0

        # Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
        inputs.odometryTimestamps = [Timer.getFPGATimestamp()]
        inputs.odometryDrivePositions = [inputs.drivePosition]
        inputs.odometrySteerPositions = [inputs.steerPosition]

    def setDriveOpenLoop(self, output: float) -> None:
        self._driveClosedLoop = False
        self._driveAppliedVolts = output

    def setSteerOpenLoop(self, output: float) -> None:
        self._steerClosedLoop = False
        self._steerAppliedVolts = output

    def setDriveVelocity(self, velocity: radians_per_second, feedforward: float) -> None:
        self._driveClosedLoop = True
        self._driveFFVolts = feedforward
        self._driveController.setSetpoint(velocity)

    def setSteerPosition(self, rotation: Rotation2d) -> None:
        self._steerClosedLoop = True
        self._steerController.setSetpoint(rotation.radians())

    def setDrivePID(self, kP: float, kI: float, kD: float) -> None:
        self._driveController.setPID(kP, kI, kD)

    def setSteerPID(self, kP: float, kI: float, kD: float) -> None:
        self._steerController.setPID(kP, kI, kD)


class Module:
    # NOTE: These comments assume Voltage-based control. Torque current feedforwards units can be found online.
    # Static feedforward for drive motor (amount of volts to *just about* overcome static friction, find via tuner)
    _drivekS: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/DrivekS")

    # Velocity feedforward, amount of volts to maintain a constant speed
    # Find using characterization or Phoenix Tuner (preferably characterization)
    _drivekV: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/DrivekV")

    # Torque feedforward, unused in voltage control.
    # This only needs tuning if using different motors than KrakenX60FOC's
    _drivekT: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/DrivekT", driveRatio / driveMotor.Kt)

    # P and D
    _drivekP: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/DrivekP")
    _drivekD: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/DrivekD")

    # Steer P and D
    _steerkP: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/TurnkP")
    _steerkD: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/Module/TurnkD")

    if Constants.currentMode == Constants.Mode.REAL:
        # Real robot, use actual constants
        _drivekS.initDefault(drivekSReal)
        _drivekV.initDefault(drivekVReal)
        _drivekP.initDefault(drivekPReal)
        _drivekD.initDefault(drivekDReal)

        _steerkP.initDefault(steerkPReal)
        _steerkD.initDefault(steerkDReal)
    else:
        # Simulating, use simulated constants
        _drivekS.initDefault(drivekSSim)
        _drivekV.initDefault(drivekVSim)
        _drivekP.initDefault(drivekPSim)
        _drivekD.initDefault(drivekDSim)
        _steerkP.initDefault(steerkPSim)
        _steerkD.initDefault(steerkDSim)

    def __init__(self, io: ModuleIO, name: str) -> None:
        self._inputs = ModuleIO.ModuleIOInputs()
        self._io = io
        self._name = name

        self._ffModel = SimpleMotorFeedforwardMeters(self._drivekS.get(), self._drivekV.get())

        self._driveDisconnectedAlert = Alert(f"Disconnected drive motor on module {name}.", Alert.AlertType.kError)
        self._steerDisconnectedAlert = Alert(f"Disconnected steer motor on module {name}.", Alert.AlertType.kError)
        self._steerEncoderDisconnectedAlert = Alert(f"Disconnected steer encoder on module {name}.", Alert.AlertType.kError)

        self._odometryPositions: List[SwerveModulePosition] = []

    def updateInputs(self) -> None:
        self._io.updateInputs(self._inputs)
        Logger.processInputs(f"Drive/Module{self._name}", self._inputs)

    def periodic(self) -> None:
        # Update tunable numbers
        if self._drivekS.hasChanged(hash(self)) or self._drivekV.hasChanged(hash(self)):
            self._ffModel = SimpleMotorFeedforwardMeters(self._drivekS.get(), self._drivekV.get())
        if self._drivekP.hasChanged(hash(self)) or self._drivekD.hasChanged(hash(self)):
            self._io.setDrivePID(self._drivekP.get(), 0, self._drivekD.get())
        if self._steerkP.hasChanged(hash(self)) or self._steerkD.hasChanged(hash(self)):
            self._io.setSteerPID(self._steerkP.get(), 0, self._steerkD.get())

        # Calculate positions for odometry
        sampleCount = len(self._inputs.odometryTimestamps)
        self._odometryPositions = []
        for i in range(sampleCount):
            positionMeters = self._inputs.odometryDrivePositions[i] * inchesToMeters(wheelRadius)
            angle = self._inputs.odometrySteerPositions[i]
            self._odometryPositions.append(SwerveModulePosition(positionMeters, angle))

        # Update alerts
        self._driveDisconnectedAlert.set(not self._inputs.driveConnected)
        self._steerDisconnectedAlert.set(not self._inputs.steerConnected)
        self._steerEncoderDisconnectedAlert.set(not self._inputs.steerEncoderConnected)

    def runSetpoint(self, state: SwerveModuleState, wheelTorqueNm: float=0.0) -> None:
        """
        Applies the desired setpoint. This assumes module states are already optimized.
        wheelTorque should only be applied when using Torque-current control.
        """

        # Apply drive velocity and cosine scalar
        speedRadPerSec = state.speed / inchesToMeters(wheelRadius)

        # Due to azimuth coupling, "back out" the speed based on steer velocity.
        if RobotBase.isReal() and couplingRatioEnable:
            speedRadPerSec += self._inputs.steerVelocity * rotationsToRadians(couplingRatio) / rotationsToRadians(driveRatio)

        self._io.setDriveVelocity(speedRadPerSec,
            self._ffModel.calculate(speedRadPerSec) +
            (
                wheelTorqueNm * self._drivekT.get()
                if driveClosedLoopOutput == ClosedLoopOutputType.TORQUE_CURRENT_FOC
                else 0
            )
        )

        # Apply steer position if greater than steer deadband
        if math.fabs((state.angle - self.getAngle()).degrees()) < steerDeadband:
            self._io.setSteerOpenLoop(0.0)
        else:
            self._io.setSteerPosition(state.angle)

    def runDriveCharacterization(self, output: float) -> None:
        """Faces all wheels forward and applies open loop voltage to drive motor."""
        self._io.setDriveOpenLoop(output)
        self._io.setSteerPosition(Rotation2d())

    def runSteerCharacterization(self, output: float) -> None:
        """Disables drive motors and applies open loop voltage to steer motor."""
        self._io.setDriveOpenLoop(0.0)
        self._io.setSteerOpenLoop(output)

    def stop(self) -> None:
        self._io.setDriveOpenLoop(0.0)
        self._io.setSteerOpenLoop(0.0)

    def getAngle(self) -> Rotation2d:
        return self._inputs.steerPosition

    def getPositionMeters(self) -> meters:
        return self._inputs.drivePosition * inchesToMeters(wheelRadius)

    def getVelocityMetersPerSec(self) -> meters_per_second:
        return self._inputs.driveVelocity * inchesToMeters(wheelRadius)

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getPositionMeters(), self.getAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getVelocityMetersPerSec(), self.getAngle())

    def getOdometryTimestamps(self) -> List[float]:
        return self._inputs.odometryTimestamps

    def getOdometryPositions(self) -> List[SwerveModulePosition]:
        return self._odometryPositions

    def getWheelRadiusCharacterizationPosition(self) -> float:
        return self._inputs.drivePosition

    def getFFCharacterizationVelocity(self) -> rotations_per_second:
        return radiansToRotations(self._inputs.driveVelocity)

    def setBrakeMode(self, enabled: bool) -> None:
        self._io.setBrakeMode(enabled)
