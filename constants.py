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

    class AutoAlignConstants:
        translationkP = 9.0
        translationkI = 0.0
        translationkD = 0.1

        headingkP = 5.0
        headingkI = 0.0
        headingkD = 0.4

    # Hardware configurations - robot-specific
    class CanIDs:
        """CAN IDs for hardware devices. Values differ between Larry and Comp."""
        pass  # Values set below based on robot
    
    class ClimberConstants:
        """Climber subsystem constants. Values may differ between robots."""
        pass  # Values set below based on robot


# Initialize robot-specific hardware configurations
def _init_hardware_configs():
    """Initialize hardware configurations based on detected robot."""
    
    # CAN IDs configuration
    if currentRobot == Robot.LARRY:
        # Larry (test robot) CAN IDs
        Constants.CanIDs.CLIMB_TALON = 10
        Constants.CanIDs.INTAKE_TALON = 11
        # Add other Larry-specific IDs here
    else:  # COMP or UNKNOWN defaults to COMP
        # Comp (competition robot) CAN IDs
        Constants.CanIDs.CLIMB_TALON = 15
        Constants.CanIDs.INTAKE_TALON = 16
        # Add other Comp-specific IDs here
    
    # Climber constants configuration
    if currentRobot == Robot.LARRY:
        # Larry (test robot) climber configuration
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
    else:  # COMP or UNKNOWN defaults to COMP
        # Comp (competition robot) climber configuration
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


# Initialize hardware configs at module load time
_init_hardware_configs()
