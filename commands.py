from dataclasses import field
from enum import Enum, auto
from typing import Callable

from commands2 import Command, cmd, ConditionalCommand
from phoenix6.swerve import ClosedLoopOutputType
from pykit.logger import Logger
from wpilib import DriverStation, Timer, RobotBase
from wpimath import applyDeadband
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile

from constants import Constants
from subsystems.drive import Drive
from subsystems.drive.module import ModuleIOTalonFX
from subsystems.toast import *
from util import LoggedTunableNumber

# Controller deadband
_deadband: Final[float] = 0.1

# Heading angle P and D
_anglekP: Final[float] = 5.0
_anglekD: Final[float] = 0.4

# Heading angle PID max vel and acceleration
_angleMaxVel: Final[float] = 8.0
_angleMaxAccel: Final[float] = 20.0

# Characterization configs
_feedForwardStartDelay: Final[float] = 2.0
_feedForwardRampRate: Final[float] = 0.1
_wheelRadiusMaxVel: Final[float] = 0.25
_wheelRadiusRampRate: Final[float] = 0.05

# Skew Compensation Scalar
# This counteracts drift when rotating and translating in teleop.

# To measure:
# 1. Drive in a straight line and rotate at max teleop speed.
# 2. Check for translational drift
# 3. Increment scalar
# 4. Repeat until drift is negligible
_skewCompensationScalar: Final[LoggedTunableNumber] = LoggedTunableNumber("Drive/SkewCompensationScalar", skewCompensationScalarDefault)

__closestBranch: Pose2d | None = None


class BranchSide(Enum):
    LEFT = auto(),
    RIGHT = auto()

_blueLeftBranches: Final[List[Pose2d]] = [
    Pose2d(3.091, 4.181, Rotation2d()),
    Pose2d(3.656, 2.916, Rotation2d.fromDegrees(60)),
    Pose2d(5.023, 2.772, Rotation2d.fromDegrees(120)),
    Pose2d(5.850, 3.851, Rotation2d.fromRotations(180)),
    Pose2d(5.347, 5.134, Rotation2d.fromDegrees(240)),
    Pose2d(3.932, 5.302, Rotation2d.fromDegrees(300))
]

_blueRightBranches: Final[List[Pose2d]] = [
    Pose2d(3.091, 3.863, Rotation2d()),
    Pose2d(3.956, 2.748, Rotation2d.fromDegrees(60)),
    Pose2d(5.323, 2.928, Rotation2d.fromDegrees(120)),
    Pose2d(5.862, 4.187, Rotation2d.fromDegrees(180)),
    Pose2d(5.047, 5.290, Rotation2d.fromDegrees(240)),
    Pose2d(3.668, 5.110, Rotation2d.fromDegrees(300))
]

def mirrorPoses(blueTargets: List[Pose2d]) -> List[Pose2d]:
    mirroredPoses = []
    fieldLength = Constants.FIELD_LAYOUT.getFieldLength()
    fieldWidth = Constants.FIELD_LAYOUT.getFieldWidth()
    for i in range(len(blueTargets)):
        pose = blueTargets[i]
        mirroredPoses.append(
            Pose2d(
                fieldLength - pose.X(),
                fieldWidth - pose.Y(),
                pose.rotation() + Rotation2d.fromDegrees(180)
            )
        )
    return mirroredPoses

_redLeftBranches = mirrorPoses(_blueLeftBranches)
_redRightBranches = mirrorPoses(_blueRightBranches)

def _getLinearVelocityFromJoysticks(x: float, y: float) -> Translation2d:
    # Apply deadband
    linearMagnitude = applyDeadband(math.hypot(x, y), _deadband)
    linearDirection = Rotation2d(math.atan2(y, x))

    # Square magnitude for more precise control
    linearMagnitude **= 2

    # Return new linear velocity
    return Pose2d(Translation2d(), linearDirection).transformBy(
        Transform2d(linearMagnitude, 0, Rotation2d())
    ).translation()

def _getChassisSpeeds(drive: Drive, omegaSupplier: Callable[[], float], linearVelocity: Translation2d) -> ChassisSpeeds:
    omega = applyDeadband(omegaSupplier(), _deadband)

    # Square rotation value for more precise control
    omega = math.copysign(omega**2, omega)

    # When translation and rotation, a slight translational skew can be noticed. This mitigates the
    # issue during teleop.
    skewCompensationFactor = Rotation2d(drive.getChassisSpeeds().omega * _skewCompensationScalar.get())

    # Get field speeds, convert to robot-relative, then convert back to field speeds with skew
    # compensated
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            ChassisSpeeds(
                linearVelocity.X() * maxLinearSpeed,
                linearVelocity.Y() * maxLinearSpeed,
                omega * maxAngularSpeed
            ), drive.getPose().rotation()
        ), drive.getPose().rotation() + skewCompensationFactor
    )

def _getClosestBranch(drive: Drive, branchSide: BranchSide) -> Pose2d:
    def getDistanceToLine(target: Pose2d) -> float:
        robot = drive.getPose()

        slope = math.tan(target.rotation().radians())
        x = (robot.X() + slope**2 * target.X()
             - slope * target.Y()
             + slope * robot.Y()
             ) / (slope**2 + 1)
        y = slope * (x - target.X()) + target.Y()

        possible = Pose2d(x, y, target.rotation())
        reefX = (4.4735
                 if (DriverStation.Alliance.kBlue if DriverStation.getAlliance() is None else DriverStation.getAlliance()) == DriverStation.Alliance.kBlue
                 else Constants.FIELD_LAYOUT.getFieldLength() - 4.4735)
        if (target.X() - reefX <= 0) and (x - reefX <= 0):
            return robot.translation().distance(possible.translation())
        return math.inf

    alliance = DriverStation.getAlliance() or DriverStation.Alliance.kBlue

    if alliance == DriverStation.Alliance.kRed:
        targets = _redLeftBranches if branchSide == BranchSide.LEFT else _redRightBranches
    else:
        targets = _blueLeftBranches if branchSide == BranchSide.LEFT else _blueRightBranches

    return min(
        targets,
        key=lambda p: getDistanceToLine(p),
        default=targets[0]
    )

def fieldRelative(drive: Drive, xSupplier: Callable[[], float], ySupplier: Callable[[], float], omegaSupplier: Callable[[], float]) -> Command:
    def run_drive():
        # Get linear velocity from joysticks
        linear_velocity = _getLinearVelocityFromJoysticks(
            xSupplier(),
            ySupplier()
        )

        # Get chassis speeds (includes rotation deadband handling)
        speeds = _getChassisSpeeds(drive, omegaSupplier, linear_velocity)

        # Check alliance for field flipping
        alliance = DriverStation.getAlliance()
        is_flipped = alliance is not None and alliance == DriverStation.Alliance.kRed

        # Adjust robot rotation if flipped
        robot_rotation = drive.getRotation()
        if is_flipped:
            robot_rotation = robot_rotation + Rotation2d.fromDegrees(180)

        # Run drivetrain
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                robot_rotation
            )
        )

    return cmd.run(run_drive, drive).withName("FieldRelative")

def robotRelative(drive: Drive, xSupplier: Callable[[], float], ySupplier: Callable[[], float], omegaSupplier: Callable[[], float]) -> Command:
    def run_drive():
        # Get linear velocity
        linearVelocity = _getLinearVelocityFromJoysticks(xSupplier(), ySupplier())
        drive.runVelocity(_getChassisSpeeds(drive, omegaSupplier, linearVelocity))

    return cmd.run(run_drive, drive).withName("RobotRelative")

def brakeWithX(drive: Drive) -> Command:
    return cmd.run(drive.stopWithX, drive).withName("BrakeX")

def alignToClosestBranch(drive: Drive, branchSide: BranchSide, xSupplier: Callable[[], float], ySupplier: Callable[[], float]) -> Command:
    translationController = PIDController(
        Constants.AutoAlignConstants.translationkP,
        Constants.AutoAlignConstants.translationkI,
        Constants.AutoAlignConstants.translationkD
    )

    rotationController = ProfiledPIDController(
        Constants.AutoAlignConstants.headingkP,
        Constants.AutoAlignConstants.headingkI,
        Constants.AutoAlignConstants.headingkD,
        TrapezoidProfile.Constraints(_angleMaxVel, _angleMaxAccel)
    )
    rotationController.enableContinuousInput(-math.pi, math.pi)

    def setup():
        global __closestBranch
        __closestBranch = _getClosestBranch(drive, branchSide)
        Logger.recordOutput("Drive/TargetBranch", __closestBranch)

    def reset():
        rotationController.reset(drive.getRotation().radians())
        translationController.reset()

    def periodic():
        currentPose = drive.getPose()
        targetRot = __closestBranch.rotation()

        # Compute Y error rotated to robot frame
        yError = (currentPose.X() * -targetRot.sin()
                  + currentPose.Y() * targetRot.cos()
                  ) - (__closestBranch.X() * -targetRot.sin()
                       + __closestBranch.Y() * targetRot.cos())

        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            yError *= -1.0

        # Rotate robot relative velocity to field-centric
        clamped = max(-1.5, min(translationController.calculate(yError, 0.0), 1.5))
        velY = applyDeadband(clamped, maxLinearSpeed * 0.01)

        fieldRelativeVelocity = Translation2d(xSupplier() * targetRot.cos() + ySupplier() * targetRot.sin(), velY).rotateBy(targetRot)

        # Heading PID
        rotationVel = rotationController.calculate(
            currentPose.rotation().radians(), targetRot.radians()
        )

        speeds = ChassisSpeeds(
            fieldRelativeVelocity.X(),
            fieldRelativeVelocity.Y(),
            rotationVel
        )

        isFlipped = DriverStation.getAlliance() is not None and DriverStation.getAlliance() == DriverStation.Alliance.kRed

        # Drive field relative
        drive.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                drive.getRotation() + Rotation2d.fromDegrees(180) if isFlipped else drive.getRotation()
            )
        )

    return cmd.runOnce(setup).andThen(
        cmd.run(periodic, drive)
    ).beforeStarting(reset).withName("AlignToClosestReedBranch")

def feedforwardCharacterization(drive: Drive) -> Command:
    """
    Measures the velocity feedforward constants for the drive motors.
    This command can only be used in voltage control mode.
    """
    velocitySamples = []
    voltageSamples = []
    timer = Timer()

    def results():
        n = len(velocitySamples)
        sumX = 0.0
        sumY = 0.0
        sumXY = 0.0
        sumX2 = 0.0
        for i in range(n):
            sumX += velocitySamples[i]
            sumY += voltageSamples[i]
            sumXY += velocitySamples[i] * voltageSamples[i]
            sumX2 += velocitySamples[i]**2
        kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX**2)
        kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX**2)

        print("********** Drive FF Characterization Results **********")
        print(f"\tkS: {kS:.5f}")
        print(f"\tkV: {kV:.5f}")

    def error():
        raise RuntimeError("Drive motor ClosedLoopOutputType must be Voltage.")

    return ConditionalCommand(
        cmd.sequence(
            # Reset data
            cmd.runOnce(
                lambda: (velocitySamples.clear(), voltageSamples.clear())
            ),

            # Allow modules to orient
            cmd.run(lambda: drive.runDriveCharacterization(0.0), drive).withTimeout(_feedForwardStartDelay),

            # Start timer
            cmd.runOnce(lambda: timer.restart()),

            # Accelerate and gather data
            cmd.run(
                lambda: (
                    drive.runDriveCharacterization(timer.get() * _feedForwardRampRate),
                    velocitySamples.append(drive.getFFCharacterizationVelocity()),
                    voltageSamples.append(timer.get() * _feedForwardRampRate)
                ),
                drive
            ).finallyDo( # When cancelled, calculate and print results
                lambda _: results()
            )
        ),
        cmd.runOnce(lambda: error()),
        lambda: ModuleIOTalonFX.driveClosedLoopOutput == ClosedLoopOutputType.VOLTAGE or RobotBase.isSimulation()
    )

def wheelRadiusCharacterization(drive: Drive) -> Command:

    @dataclass
    class WheelRadiusCharacterizationState:
        positions: List[float] = field(default_factory=list)
        lastAngle: Rotation2d = field(default_factory=Rotation2d)
        gyroDelta: float = 0.0

    limiter = SlewRateLimiter(_wheelRadiusRampRate)
    state = WheelRadiusCharacterizationState()

    def record_start():
        state.positions = drive.getWheelRadiusCharacterizationPositions()
        state.lastAngle = drive.getRotation()
        state.gyroDelta = 0.0

    def update():
        rotation = drive.getRotation()
        state.gyroDelta += abs((rotation - state.lastAngle).radians())
        state.lastAngle = rotation

    def results():
        positions = drive.getWheelRadiusCharacterizationPositions()
        wheelDelta = 0.0
        n = len(Drive.getModuleTranslations())
        for i in range(n):
            wheelDelta += abs(positions[i] - state.positions[i]) / float(n)
        wheelRadius = (state.gyroDelta * driveBaseRadius) / wheelDelta
        print("********** Wheel Radius Characterization Results **********")
        print(f"\tWheel Delta: {wheelDelta:.3f} radians")
        print(f"\tGyro Delta: {state.gyroDelta:.3f} radians")
        print(f"\tWheel Radius: {wheelRadius:.3f} meters / {metersToInches(wheelRadius):.3f} inches")

    return cmd.parallel(
        # Drive control sequence
        cmd.sequence(
            # Reset acceleration limiter
            cmd.runOnce(lambda: limiter.reset(0.0)),

            # Turn in place, accelerating up to full speed
            cmd.run(
                lambda: drive.runVelocity(ChassisSpeeds(0.0, 0.0, limiter.calculate(_wheelRadiusMaxVel))),
                drive
            )
        ),

        # Measurement sequence
        cmd.sequence(
            # Wait for modules to fully orient before starting measurement
            cmd.waitSeconds(1.0),

            # Record stating measurement
            cmd.runOnce(record_start),

            # Update gyro delta. When cancelled, calculate and print results
            cmd.run(update).finallyDo(lambda _: results)
        )
    ).withName("WheelRadiusCharacterization")
