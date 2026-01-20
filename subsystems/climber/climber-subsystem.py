"""
ClimberSubsystem using PyKit IO layer pattern.
"""

from enum import auto, Enum
from typing import Final

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import rotationsToRadians

from constants import Constants
from subsystems import StateSubsystem
from subsystems.climber.io import ClimberIO


@autologgable_output
class ClimberSubsystem(StateSubsystem):
    """
    The ClimberSubsystem is responsible for controlling the robot's climber mechanism.
    Uses PyKit IO layer for hardware abstraction.
    """

    class SubsystemState(Enum):
        STOP = auto()
        # STOW = auto()
        # EXTEND = auto()

    _state_configs: dict[SubsystemState, tuple[float, float]] = {
        SubsystemState.STOP: (0.0, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE)
        # Add other state configs as needed
        # SubsystemState.CLIMB_IN: (Constants.ClimberConstants.VOLTAGE_INWARDS, Constants.ClimberConstants.SERVO_ENGAGED_ANGLE)
        # SubsystemState.CLIMB_OUT: (Constants.ClimberConstants.VOLTAGE_OUTWARDS, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE)
    }

    def __init__(self, io: ClimberIO) -> None:
        """
        Initialize the climber subsystem.

        :param io: The climber IO implementation (ClimberIOTalonFX for real hardware, ClimberIOSim for simulation)
        """
        super().__init__("Climber", self.SubsystemState.STOP)
        
        self._io: Final[ClimberIO] = io
        self._inputs = ClimberIO.ClimberIOInputs()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Climber motor is disconnected.", Alert.AlertType.kError)

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Climber", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        """
        Set the desired climber state.

        :param desired_state: The desired state to transition to
        """
        if not super().set_desired_state(desired_state):
            return

        # Get motor voltage and servo angle for this state
        motor_voltage, servo_angle = self._state_configs.get(
            desired_state, 
            (0.0, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE)
        )
        
        # Set motor voltage through IO layer
        self._io.setMotorVoltage(motor_voltage)
        
        # Set servo angle through IO layer
        self._io.setServoAngle(servo_angle)

    def get_position(self) -> float:
        """
        Get the current climber position.

        :return: Position in rotations (adjust with gear ratio as needed)
        """
        # Convert from radians to rotations, then apply gear ratio
        return self._inputs.motorPosition / (2 * 3.14159) * Constants.ClimberConstants.GEAR_RATIO

    def get_component_pose(self) -> Pose3d:
        """
        Get the 3D pose of the climber component.

        :return: Pose3d representing the climber's position and orientation
        """
        return Pose3d(
            0, 
            0.292100, 
            0.463550, 
            Rotation3d(rotationsToRadians(self.get_position()), 0, 0)
        )
