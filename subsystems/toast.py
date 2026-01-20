from dataclasses import dataclass
from typing import Final, List

import pathplannerlib.config
from pathplannerlib.config import RobotConfig
from phoenix6.swerve import ClosedLoopOutputType
from wpimath.geometry import Translation2d
from wpimath.system.plant import DCMotor
from wpimath.units import *

"""***DRIVETRAIN***"""

# Odometry frequency of the odometry thread. 75hz for roboRIO CAN, 250hz if using a CANivore (CANFD)
odomFrequency: Final[hertz] = 250

# Gyro ID
gyroId: Final[int] = 9

# Distance between 2 wheels (horizontally or vertically)
trackWidthMeters = inchesToMeters(24.0)

# THEORETICAL max free speed of the robot. This is the robot travelling max speed in one direction.
# Easiest way to measure this is to check a wheel's highest rot/s in Phoenix Tuner then multiply by the wheel's circumference (in meters).
maxLinearSpeed: Final[meters_per_second] = 4.2

# Maximum speed a swerve module can rotate in radians per second.
# Used for SwerveSetpointGenerator.
maxWheelRotationSpeed: Final[radians_per_second] = rotationsToRadians(3)

# Minimum degree change needed before the steer motors move. 0.3 by default.
steerDeadband: Final[degrees] = 0.3

# Wheel radius in inches.
# Use the characterization test periodically to update this value through the course of an event.
wheelRadius: Final[inches] = 2

# Mass of the robot in kilograms.
# If you can't physically measure, take CAD estimate and add 10% of robot weight for wires.
robotMass: Final[kilograms] = 74.088

# MOI of the robot in kg^2/m. Personally, I take the CAD's estimate (with game pieces) and round up to account for wires.
robotMOI: Final[kilogram_square_meters] = 6.883

# Coefficient of friction for tread. This can probably be measured somehow, but 1.2 works well for SDS tread.
wheelCOF: Final[float] = 1.2

### MOTOR CONSTANTS

# DCMotor for the drive motor.
driveMotor = DCMotor.krakenX60FOC(1)

# DCMotor for the steer motor.
steerMotor = DCMotor.krakenX60FOC(1)

# Gear reductions
driveRatio: Final[float] = 6.746031746031747
steerRatio: Final[float] = 21.428571428571427

# Ratio of 1 azimuth rotation to drive motor rotations. Find via characterization.
couplingRatio: Final[float] = 3.5714285714285716
couplingRatioEnable : Final[bool] = True # If it doesn't work turn this to False

# Stator limits (limits max acceleration, used to prevent brownouts)
driveStatorLimit: Final[amperes] = 80
steerStatorLimit: Final[amperes] = 60

# Loop outputs (Determines PID units and other... stuff ig)
driveClosedLoopOutput: Final[ClosedLoopOutputType] = ClosedLoopOutputType.VOLTAGE
steerClosedLoopOutput: Final[ClosedLoopOutputType] = ClosedLoopOutputType.VOLTAGE


### MODULE CONFIGS
@dataclass
class ModuleConfig:
    steerMotorId: int
    driveMotorId: int
    encoderId: int
    encoderOffsetRotations: float
    isDriveInverted: bool
    isSteerInverted: bool
    isEncoderInverted: bool

moduleConfigs: Final[List[ModuleConfig]] = [
    # Front Left
    ModuleConfig(
        3, # Steer motor ID
        7, # Drive motor ID
        7, # Encoder ID
        0.155029296875, # Encoder offset (zero encoders in Phoenix Tuner)
        True, # Invert drive motor (use if driving backwards)
        False, # Invert steer motors (use if steering in reverse)
        False # Invert encoder motors (use if steer motor position isn't working properly)
    ),
    # Front Right
    ModuleConfig(
        1, 5, 5, -0.27978515625, True, False, False
    ),
    # Back Left
    ModuleConfig(
        4, 8, 8, -0.241943359375, True, False, False
    ),
    # Back Right
    ModuleConfig(
        2, 6, 6, -0.06494140625, True, False, False
    )
]


"""***VISION***"""

# Basic filtering thresholds
maxAmbiguity: Final[float] = 0.3
maxZError: Final[float] = 0.75

# Standard deviation baselines, for 1-meter distance and 1 tag
# (Adjusted automatically based on distance and # of tags)
linearStdDevBaseline: Final[meters] = 0.02
angularStdDevBaseline: Final[radians] = 0.06

# Standard deviations multipliers for each camera
# (Adjust to trust some cameras more than others
cameraStdDevFactors: Final[tuple[float, float, float]] = (1.0, 0.8, 0.8)

# Multipliers to apply for MegaTag 2 observations
linearStdDevMegatag2Factor: Final[float] = 0.5
angularStdDevMegatag2Factor: Final[float] = math.inf # Ignore because the robot angle is given to LL


"""***TUNING CONSTANTS***"""
# All of these values should be tuned via tuningMode or PhoenixTuner.
# When tuning swerve modules, tune them while on carpet for accurate results.

### Drivetrain

## Coasting
# Amount of time to wait after not moving while disabled to enable coast mode.
coastWaitTimeDefault: Final[seconds] = 0.5

# Minimum speed to begin counting down to enabling coast mode (see above)
coastMetersPerSecondThreshold: Final[meters_per_second] = 0.05

## Anti-tip
# Anti-tip master switch.
antiTipEnabled: Final[bool] = False

# kP for Anti-tip control. Be careful when testing.
tipKpDefault: Final[degrees] = 0.03

# Minimum amount of degrees to enable tip control.
tipThresholdDefault: Final[degrees] = 3

## Pathplanner PID
# Tune if error handling in auto is unsatisfactory.
ppTranslationPID = (5, 0, 0)
ppRotationPID = (5, 0, 0)

### Modules

## REAL PID
# PID constants for the physical robot.
# These can be easily tuned through Elastic.
drivekSReal: Final[volts | amperes] = 0.141
drivekVReal: Final[volts | amperes] = 0.82
drivekPReal: Final[volts | amperes] = 0.2
drivekDReal: Final[volts | amperes] = 0.0

steerkPReal: Final[volts | amperes] = 100
steerkDReal: Final[volts | amperes] = 0.3


## SIM PID
# PID constants for the robot in simulation.
drivekSSim: Final[volts | amperes] = 0.03
drivekVSim: Final[volts | amperes] = 1.0 / rotationsToRadians(1.0 / 0.91035)
drivekPSim: Final[volts | amperes] = 0.1
drivekDSim: Final[volts | amperes] = 0.0

steerkPSim: Final[volts | amperes] = 10
steerkDSim: Final[volts | amperes] = 0

## COMMANDS

# Skew Compensation Scalar (teleop only)
# This counteracts drift when rotating and translating in teleop.

# To measure:
# 1. Drive in a straight line and rotate at max teleop speed.
# 2. Check for translational drift
# 3. Increment scalar (via redeploy or Elastic tuning)
# 4. Repeat until drift is negligible
skewCompensationScalarDefault: float = 0




#######################################################################################################
### DO NOT EDIT VARIABLES BELOW THIS LINE
# These are calculated automatically.
moduleTranslations: Final[List[Translation2d]] = [
    Translation2d(trackWidthMeters / 2, trackWidthMeters / 2),
    Translation2d(trackWidthMeters / 2, -trackWidthMeters / 2),
    Translation2d(-trackWidthMeters / 2, trackWidthMeters / 2),
    Translation2d(-trackWidthMeters / 2, -trackWidthMeters / 2)
]

pathPlannerConfig: Final[RobotConfig] = RobotConfig(
    robotMass,
    robotMOI,
    pathplannerlib.config.ModuleConfig(
        inchesToMeters(wheelRadius),
        maxLinearSpeed,
        wheelCOF,
        driveMotor.withReduction(driveRatio),
        driveStatorLimit,
        1),
    moduleTranslations
)

driveBaseRadius: Final[meters] = trackWidthMeters / math.sqrt(2) # Assumes square drivetrain.
maxAngularSpeed: Final[radians_per_second] = maxLinearSpeed / driveBaseRadius
