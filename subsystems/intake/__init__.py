from enum import auto, Enum

from commands2 import Command, cmd
from phoenix6 import utils
from phoenix6.configs import CANrangeConfiguration, TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, HardwareLimitSwitchConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX, CANrange
from phoenix6.signals import NeutralModeValue, ForwardLimitValue, ForwardLimitSourceValue

from pykit.autolog import autologgable_output
from pykit.logger import Logger
from wpilib import Alert
from typing import Final
from constants import Constants
from subsystems import StateSubsystem
from subsystems.intake.io import IntakeIO


class IntakeSubsystem(StateSubsystem):
    """
    The IntakeSubsystem is responsible for controlling the end effector's compliant wheels.
    """

    class SubsystemState(Enum):
        STOP = auto()
        INTAKE = auto()
        OUTPUT = auto()

    _state_configs: dict[SubsystemState, float] = {
        SubsystemState.STOP: 0,
        SubsystemState.INTAKE: 12.0,
        SubsystemState.OUTPUT: -12.0
    }

    def __init__(self, io: IntakeIO) -> None:
        super().__init__("Intake", self.SubsystemState.STOP)

        self._io: Final[IntakeIO] = io
        self._inputs = IntakeIO.IntakeIOInputs()
        
        # Alert for disconnected motor
        self._motorDisconnectedAlert = Alert("Intake motor is disconnected.", Alert.AlertType.kError)

    def periodic(self) -> None:
        """Called periodically to update inputs and log data."""
        # Update inputs from hardware/simulation
        self._io.updateInputs(self._inputs)
        
        # Log inputs to PyKit
        Logger.processInputs("Intake", self._inputs)
        
        # Update alerts
        self._motorDisconnectedAlert.set(not self._inputs.motorConnected)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            return

        # Get motor voltage for this state
        motor_voltage = self._state_configs.get(
            desired_state, 
            (0.0)
        )
        
        # Set motor voltage through IO layer
        self._io.setMotorVoltage(motor_voltage)

    