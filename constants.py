from enum import Enum, auto
from typing import Final

from phoenix6.configs.config_groups import Slot0Configs
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import RobotBase

from robot_config import currentRobot, Robot


class Constants:
    tuningMode: Final[bool] = False

    class Mode(Enum):
        # Running on a real robot.
        REAL = auto(),

        # Running a physics simulator.
        SIM = auto(),

        # Replaying from a log file.
        REPLAY = auto()

    simMode: Final[Mode] = Mode.SIM
    currentMode: Final[Mode] = Mode.REAL if RobotBase.isReal() else simMode

    FIELD_LAYOUT: Final[AprilTagFieldLayout] = AprilTagFieldLayout.loadField(AprilTagField.k2025ReefscapeWelded)

    # Hardware configurations
    # Can ids are to be set in the same order as they are wired in the CAN bus
    class CanIDs:
        #All motors are Kraken X60 unless otherwise specified
        CLIMB_TALON = 10
        INTAKE_TALON = 11
        KICKER_TALON = 12
        TURRET_TALON = 13
        HOOD_TALON = 14 # Kraken X44
        LAUNCHER_LEFT_TALON = 15 # Kraken X44
        LAUNCHER_RIGHT_TALON = 16 # Kraken X44
        TURRET_CANCODER = 17
        HOOD_CANCODER = 18
    
    """Values may differ between robots."""
    class ClimberConstants:
        GEAR_RATIO = None
        GAINS = None
        SERVO_PORT = None
        SERVO_ENGAGED_ANGLE = None
        VOLTAGE_INWARDS = None
        SERVO_DISENGAGED_ANGLE = None
        VOLTAGE_OUTWARDS = None
        CLIMB_FULL_THRESHOLD = None

    class IntakeConstants:
        GEAR_RATIO = None
        GAINS = None
        SUPPLY_CURRENT = None
        INSIDE_FRAME_ANGLE = None

    class LauncherConstants:
        GEAR_RATIO = None
        GAINS = None
        SUPPLY_CURRENT = None
        INSIDE_FRAME_ANGLE = None

    class FeederConstants:
        GEAR_RATIO = None
        GAINS = None
        SUPPLY_CURRENT = None
        INSIDE_FRAME_ANGLE = None

    class VisionConstants:
        FRONT = "limelight-front"
        # LAUNCHER = "limelight-al"



# Initialize robot-specific hardware configurations
def _init_hardware_configs():
    """Initialize hardware configurations based on detected robot."""

    match currentRobot:
        case Robot.LARRY:
            # Climber
            Constants.ClimberConstants.GEAR_RATIO = 61504.0 / 189
            Constants.ClimberConstants.GAINS = (Slot0Configs()
                .with_k_p(1.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.ClimberConstants.SERVO_PORT = 0
            Constants.ClimberConstants.SERVO_ENGAGED_ANGLE = 0.0
            Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE = 90.0
            Constants.ClimberConstants.VOLTAGE_INWARDS = 16.0
            Constants.ClimberConstants.VOLTAGE_OUTWARDS = -4.0
            Constants.ClimberConstants.CLIMB_FULL_THRESHOLD = 100.0  # Adjust as needed

        case _:  # COMP or UNKNOWN defaults to COMP
            # Climber
            Constants.ClimberConstants.GEAR_RATIO = 61504.0 / 189  # Same or different?
            Constants.ClimberConstants.GAINS = (Slot0Configs()
                .with_k_p(1.0)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.ClimberConstants.SERVO_PORT = 0
            Constants.ClimberConstants.SERVO_ENGAGED_ANGLE = 0.0
            Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE = 90.0
            Constants.ClimberConstants.VOLTAGE_INWARDS = 16.0
            Constants.ClimberConstants.VOLTAGE_OUTWARDS = -4.0
            Constants.ClimberConstants.CLIMB_FULL_THRESHOLD = 100.0  # Adjust as needed
    
            # Intake
            Constants.IntakeConstants.GEAR_RATIO = 1.0  # Adjust based on actual gear ratio
            Constants.IntakeConstants.GAINS = (Slot0Configs()
                .with_k_p(0.1)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.IntakeConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.IntakeConstants.INSIDE_FRAME_ANGLE = 45.0  # Degrees (example value)

            # Launcher
            Constants.LauncherConstants.GEAR_RATIO = 1.0  # Adjust based on actual gear ratio
            Constants.LauncherConstants.GAINS = (Slot0Configs()
                .with_k_p(0.1)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.LauncherConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.LauncherConstants.INSIDE_FRAME_ANGLE = 45.0  # Degrees (example value)

            # Feeder
            Constants.FeederConstants.GEAR_RATIO = 1.0  # Adjust based on actual gear ratio
            Constants.FeederConstants.GAINS = (Slot0Configs()
                .with_k_p(0.1)
                .with_k_i(0.0)
                .with_k_d(0.0)
                .with_k_s(0.0)
                .with_k_v(0.0)
                .with_k_a(0.0)
            )
            Constants.FeederConstants.SUPPLY_CURRENT = 30.0  # Amperes
            Constants.FeederConstants.INSIDE_FRAME_ANGLE = 45.0  # Degrees (example value)

# Initialize hardware configs at module load time
_init_hardware_configs()
