from enum import Enum, auto
from typing import Callable, Tuple

from commands2 import Subsystem, Command
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.logging import PathPlannerLogging
from pathplannerlib.pathfinders import LocalADStar
from pathplannerlib.pathfinding import Pathfinding
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.util.swerve import SwerveSetpointGenerator, SwerveSetpoint
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib import Timer, DriverStation, Alert
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Pose3d, Twist3d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveModulePosition

from subsystems.drive.gyro import GyroIO
from subsystems.drive.module import ModuleIO, Module
from subsystems.toast import *
from util import LoggedTunableNumber, LoggedTracer, PhoenixOdometryThread


@autologgable_output
class Drive(Subsystem):
    # Amount of time to wait before engaging coast mode
    coastWaitTime: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/CoastWaitTimeSeconds", coastWaitTimeDefault)

    # Minimum speed before counting to coastWaitTime
    coastMetersPerSecondThreshold: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", coastMetersPerSecondThreshold)

    # P value for tip (IN DEGREES, NOT RADIANS)
    tipKp: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/AntiTip/TipKp", tipKpDefault)

    # Minimum degrees before engaging anti-tip. Reminder: Anti-tip overrides ALL inputs, in teleop and auto. Be conservative.
    tipThreshold: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/AntiTip/TipThreshold", tipThresholdDefault)

    class CoastRequest(Enum):
        AUTOMATIC = auto()
        ALWAYS_BRAKE = auto()
        ALWAYS_COAST = auto()

    def __init__(self, gyroIO: GyroIO, flModuleIO: ModuleIO, frModuleIO: ModuleIO, blModuleIO: ModuleIO, brModuleIO: ModuleIO, resetSimulationPoseCallback: Callable[[Pose2d], None]) -> None:
        super().__init__()
        self._gyroIO: Final[GyroIO] = gyroIO
        self._gyroInputs = GyroIO.GyroIOInputs()
        self._modules: Final[Tuple[Module, Module, Module, Module]] = (
            Module(flModuleIO, "FrontLeft"),
            Module(frModuleIO, "FrontRight"),
            Module(blModuleIO, "BackLeft"),
            Module(brModuleIO, "BackRight")
        )
        self.setName("Drivetrain")

        self._gyroDisconnectedAlert = Alert("Disconnected gyro, using kinematics.", Alert.AlertType.kError)

        self._resetSimulationPoseCallback = resetSimulationPoseCallback

        self._lastMovementTimer = Timer()
        self._lastMovementTimer.start()

        self._brakeModeEnabled = True

        self._isTipping = False

        self._antiTipSpeeds = ChassisSpeeds()

        self._kinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(*self.getModuleTranslations())

        self._rawGyroRotation = Rotation2d()
        self._lastModulesPositions = [SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition(), SwerveModulePosition()] # For delta tracking
        self._poseEstimator: Final[SwerveDrive4PoseEstimator] = SwerveDrive4PoseEstimator(self._kinematics, self._rawGyroRotation, self._lastModulesPositions, Pose2d())

        # Start odometry thread
        PhoenixOdometryThread.getInstance().start()

        # Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            self.getPose,
            self.setPose,
            self.getChassisSpeeds,
            lambda speeds, feedforwards: self.runVelocity(speeds, feedforwards),
            PPHolonomicDriveController(
                PIDConstants(ppTranslationPID[0], ppTranslationPID[1], ppTranslationPID[2]), PIDConstants(ppRotationPID[0], ppRotationPID[1], ppRotationPID[2])
            ),
            pathPlannerConfig,
            lambda: (DriverStation.getAlliance() == DriverStation.Alliance.kRed) if DriverStation.getAlliance() is not None else False,
            self
        )
        Pathfinding.setPathfinder(LocalADStar())
        PathPlannerLogging.setLogActivePathCallback(
            lambda activePath: Logger.recordOutput("PathPlanner/ActivePath", activePath))
        PathPlannerLogging.setLogTargetPoseCallback(
            lambda targetPose: Logger.recordOutput("PathPlanner/TargetPose", targetPose))

        self._setpointGenerator = SwerveSetpointGenerator(pathPlannerConfig, maxWheelRotationSpeed)
        self._currentSetpoint = SwerveSetpoint(self.getChassisSpeeds(), list(self._getModuleStates()), DriveFeedforwards.zeros(4))

        # Configure SysId
        self._sysId: Final[SysIdRoutine] = SysIdRoutine(
            SysIdRoutine.Config(recordState=lambda state: Logger.recordOutput("Drive/SysIdState", str(state))),
            SysIdRoutine.Mechanism(lambda voltage: self.runDriveCharacterization(voltage), lambda _: None, self)
        )

        self._coastRequest = self.CoastRequest.AUTOMATIC

    def periodic(self) -> None:
        with PhoenixOdometryThread.odometryLock:
            self._gyroIO.updateInputs(self._gyroInputs)
            Logger.processInputs("Drive/Gyro", self._gyroInputs)
            for m in self._modules:
                m.updateInputs()
        LoggedTracer.record("Drive/Inputs")

        if self.getCurrentCommand() is not None:
            Logger.recordOutput("Drive/CurrentCommand", self.getCurrentCommand().getName())
        else:
            Logger.recordOutput("Drive/CurrentCommand", "None")

        # Call periodic
        for m in self._modules:
            m.periodic()

        # Stop moving when disabled
        if DriverStation.isDisabled():
            for m in self._modules:
                m.stop()

        # Log empty setpoint values when disabled
        if DriverStation.isDisabled():
            Logger.recordOutput("Drive/States/Setpoints", [])
            Logger.recordOutput("Drive/States/SetpointsOptimized", [])

        # Update odometry
        sampleTimestamps = self._modules[0].getOdometryTimestamps() # All signals are sampled together
        for i in range(len(sampleTimestamps)):
            # Read wheel positions and deltas from each module
            modulePositions = []
            moduleDeltas = []
            for mi in range(4):
                modulePositions.append(self._modules[mi].getOdometryPositions()[i])
                moduleDeltas.append(SwerveModulePosition(modulePositions[mi].distance - self._lastModulesPositions[mi].distance, modulePositions[mi].angle))
                self._lastModulesPositions[mi] = modulePositions[mi]

            # Update gyro angle
            if self._gyroInputs.connected:
                # Use the real gyro angle
                self._rawGyroRotation = self._gyroInputs.odometryYawPositions[i]
            else:
                # Use the angle delta from the kinematics and module deltas
                twist = self._kinematics.toTwist2d(tuple(moduleDeltas))
                self._rawGyroRotation = self._rawGyroRotation + Rotation2d(twist.dtheta)

            # Apply update
            self._poseEstimator.updateWithTime(sampleTimestamps[i], self._rawGyroRotation, tuple(modulePositions))

        # Log 3D robot pose
        Logger.recordOutput("Drive/EstimatedPosition3d", Pose3d(self.getPose()).exp(
            Twist3d(0, 0, abs(self._gyroInputs.pitchPosition.radians())*trackWidthMeters/2, 0, self._gyroInputs.pitchPosition.radians(), 0)
        ).exp(
            Twist3d(0, 0, abs(self._gyroInputs.rollPosition.radians())*trackWidthMeters/2, self._gyroInputs.rollPosition.radians(), 0, 0)
        ))

        # Calculate anti-tip control
        self._isTipping = abs(self._gyroInputs.pitchPosition.degrees()) > self.tipThreshold.get() or abs(self._gyroInputs.rollPosition.degrees()) > self.tipThreshold.get()

        # Tilt direction
        tiltDirection = Rotation2d(math.atan2(-self._gyroInputs.rollPosition.degrees(), -self._gyroInputs.pitchPosition.degrees()))

        correctionSpeed = self.tipKp.get() * -math.hypot(self._gyroInputs.pitchPosition.degrees(), self._gyroInputs.rollPosition.degrees())
        correctionSpeed = max(-maxLinearSpeed/2, min(correctionSpeed, maxLinearSpeed/2))

        # Correction vector (field-relative)
        correctionVector = Translation2d(0, 1).rotateBy(tiltDirection) * correctionSpeed

        self._antiTipSpeeds = ChassisSpeeds(correctionVector.Y(), correctionVector.X(), 0)

        # Update brake mode
        # Reset movement timer if velocity above threshold
        if any([abs(m.getVelocityMetersPerSec()) > self.coastMetersPerSecondThreshold.get() for m in self._modules]):
            self._lastMovementTimer.reset()

        if DriverStation.isEnabled() and DriverStation.isFMSAttached():
            self._coastRequest = self.CoastRequest.ALWAYS_BRAKE

        match self._coastRequest:
            case self.CoastRequest.AUTOMATIC:
                if DriverStation.isEnabled():
                    self._setBrakeMode(True)
                elif self._lastMovementTimer.hasElapsed(self.coastWaitTime.get()):
                    self._setBrakeMode(False)
            case self.CoastRequest.ALWAYS_BRAKE:
                self._setBrakeMode(True)
            case self.CoastRequest.ALWAYS_COAST:
                self._setBrakeMode(False)

        # Update gyro alert
        self._gyroDisconnectedAlert.set(not self._gyroInputs.connected)
        LoggedTracer.record("Drive/Periodic")

    def _setBrakeMode(self, enabled: bool) -> None:
        if self._brakeModeEnabled != enabled:
            [m.setBrakeMode(enabled) for m in self._modules]
        self._brakeModeEnabled = enabled

    def runVelocity(self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards|None=None) -> None:
        # Override input if we're tipping
        if self._isTipping and antiTipEnabled:
            speeds = self._antiTipSpeeds

        # Only allow rotating in place in test mode (for characterization)
        if DriverStation.isTest():
            speeds.vx = 0.0
            speeds.vy = 0.0

        # Calculate module setpoints
        setpointStatesUnoptimized = self._kinematics.toSwerveModuleStates(speeds)
        self._currentSetpoint = self._setpointGenerator.generateSetpoint(self._currentSetpoint, speeds, 0.02)
        setpointStates = self._kinematics.desaturateWheelSpeeds(tuple(self._currentSetpoint.module_states), maxLinearSpeed)

        # Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drive/States/SetpointsUnoptimized", setpointStatesUnoptimized)
        Logger.recordOutput("Drive/States/Setpoints", setpointStates)
        Logger.recordOutput("Drive/ChassisSpeeds/Setpoints", self._currentSetpoint.robot_relative_speeds)

        # Send setpoints to modules
        if driveClosedLoopOutput == ClosedLoopOutputType.VOLTAGE or feedforwards is None:
            for i in range(4):
                wheelAngle = self._getModuleStates()[i].angle
                setpointStates[i].optimize(wheelAngle)
                setpointStates[i].cosineScale(wheelAngle)
                self._modules[i].runSetpoint(setpointStates[i])
        else:
            wheelForces = []
            moduleStates = self._getModuleStates()
            for i in range(4):
                # Optimize state
                wheelAngle = moduleStates[i].angle
                setpointStates[i].optimize(wheelAngle)
                setpointStates[i].cosineScale(wheelAngle)

                # Calculate wheel torque in direction
                wheelForce = feedforwards.forcesNewtons[i]
                wheelTorqueNm = wheelForce * inchesToMeters(wheelRadius)
                self._modules[i].runSetpoint(setpointStates[i], wheelTorqueNm)

                wheelForces.append(SwerveModuleState(wheelTorqueNm, setpointStates[i].angle))
            Logger.recordOutput("Drive/States/ModuleForces", wheelForces)

        # Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("Drive/States/SetpointsOptimized", setpointStates)

    def runDriveCharacterization(self, output: float) -> None:
        [m.runDriveCharacterization(output) for m in self._modules]

    def runSteerCharacterization(self, output: float) -> None:
        [m.runSteerCharacterization(output) for m in self._modules]

    def stop(self) -> None:
        self.runVelocity(ChassisSpeeds())

    def stopWithX(self) -> None:
        headings = []
        for i in range(4):
            headings.append(self.getModuleTranslations()[i].angle())
        self._kinematics.resetHeadings(tuple(headings))
        self.stop()

    def sysIdQuasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.run(lambda: self.runDriveCharacterization(0.0)).withTimeout(1).andThen(self._sysId.quasistatic(direction))

    def sysIdDynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self.run(lambda: self.runDriveCharacterization(0.0)).withTimeout(1).andThen(self._sysId.dynamic(direction))

    @autolog_output("Drive/States/Measured")
    def _getModuleStates(self) -> tuple[SwerveModuleState, ...]:
        """Returns the module states (steer angles and drive velocities) for all the modules."""
        return tuple(m.getState() for m in self._modules)

    def _getModulePositions(self) -> tuple[SwerveModulePosition, ...]:
        return tuple(m.getPosition() for m in self._modules)

    @autolog_output("Drive/ChassisSpeeds/Measured")
    def getChassisSpeeds(self) -> ChassisSpeeds:
        """Returns the measured chassis speeds of the robot."""
        return self._kinematics.toChassisSpeeds(self._getModuleStates())

    def getWheelRadiusCharacterizationPositions(self) -> List[float]:
        """Returns the position of each module in radians."""
        return [m.getWheelRadiusCharacterizationPosition() for m in self._modules]

    def getFFCharacterizationVelocity(self) -> float:
        """Returns the average velocity of the modules in rotations/sec (Phoenix native units)."""
        output = 0.0
        for m in self._modules:
            output += m.getFFCharacterizationVelocity() / 4.0
        return output

    @autolog_output("Drive/EstimatedPosition")
    def getPose(self) -> Pose2d:
        return self._poseEstimator.getEstimatedPosition()

    @autolog_output("Drive/BrakeModeEnabled")
    def _getBrakeModeEnabled(self) -> bool:
        return self._brakeModeEnabled

    @autolog_output("Drive/IsTipping")
    def _getIsTipping(self) -> bool:
        return self._isTipping

    def getRotation(self) -> Rotation2d:
        return self.getPose().rotation()

    def setPose(self, pose: Pose2d) -> None:
        self._resetSimulationPoseCallback(pose)
        self._poseEstimator.resetPosition(self._rawGyroRotation, self._getModulePositions(), pose)

    def addVisionMeasurement(self, visionRobotPoseMeters: Pose2d, timestampSeconds: float, visionMeasurementStdDevs: tuple[float, float, float]) -> None:
        """Adds a new timestamped vision measurement."""
        self._poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)

    @staticmethod
    def getModuleTranslations() -> List[Translation2d]:
        return moduleTranslations
