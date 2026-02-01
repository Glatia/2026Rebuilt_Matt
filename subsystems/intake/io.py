import math

from abc import ABC
from dataclasses import dataclass, field
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VoltageOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, volts, amperes, celsius, degrees, rotationsToRadians
from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile

from wpilib import PWMSparkMax
from wpilib.simulation import DCMotorSim
from wpimath.system import plant

from constants import Constants
from util import tryUntilOk


class IntakeIO(ABC):
    """
    Abstract base class for intake IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class IntakeIOInputs:
        """Inputs from the intake hardware/simulation."""
        # Motor status
        motorConnected: bool = False
        motorPosition: radians = 0.0
        motorVelocity: radians_per_second = 0.0
        motorAppliedVolts: volts = 0.0
        motorCurrent: amperes = 0.0
        motorTemperature: celsius = 0.0


    def updateInputs(self, inputs: IntakeIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage."""
        pass


class IntakeIOTalonFX(IntakeIO):
    """
    Real hardware implementation using TalonFX motor controller.
    """

    def __init__(self, motor_id: int, motor_config: TalonFXConfiguration) -> None:
        """
        Initialize the real hardware IO.

        :param motor_id: CAN ID of the TalonFX motor
        :param motor_config: TalonFX configuration to apply
        """
        self._motor: Final[TalonFX] = TalonFX(motor_id, "*")
        # Apply motor configuration
        tryUntilOk(5, lambda: self._motor.configurator.apply(motor_config, 0.25))
        tryUntilOk(5, lambda: self._motor.set_position(0, 0.25))

        # Create status signals for motor
        self._position: Final = self._motor.get_position()
        self._velocity: Final = self._motor.get_velocity()
        self._appliedVolts: Final = self._motor.get_motor_voltage()
        self._current: Final = self._motor.get_stator_current()
        self._temperature: Final = self._motor.get_device_temp()

        # Configure update frequencies
        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )
        self._motor.optimize_bus_utilization()

        # Voltage control request
        self._voltageRequest: Final[VoltageOut] = VoltageOut(0)

    def updateInputs(self, inputs: IntakeIO.IntakeIOInputs) -> None:
        """Update inputs with current motor state."""
        # Refresh all motor signals
        motorStatus = BaseStatusSignal.refresh_all(
            self._position,
            self._velocity,
            self._appliedVolts,
            self._current,
            self._temperature
        )

        # Update motor inputs
        inputs.motorConnected = motorStatus.is_ok()
        inputs.motorPosition = self._position.value_as_double
        inputs.motorVelocity = self._velocity.value_as_double
        inputs.motorAppliedVolts = self._appliedVolts.value_as_double
        inputs.motorCurrent = self._current.value_as_double
        inputs.motorTemperature = self._temperature.value_as_double

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage."""
        self._voltageRequest.output = voltage
        self._motor.set_control(self._voltageRequest)



class IntakeIOSim(IntakeIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self.simMotor = PWMSparkMax(1) # PLACEHOLDER

        self._motorType = plant.DCMotor.krakenX60(0) # PLACEHOLDER
        self._motorPosition: float = 0.0
        self._motorVelocity: float = 0.0
        self._motorAppliedVolts: float = 0.0

        self._intakeSim = DCMotorSim(self._motorType, Constants.IntakeConstants.GEAR_RATIO, 0.5) # PLACEHOLDER
        self._closedLoop = False

        self._controller = PIDController(
            Constants.IntakeConstants.GAINS.kP,
            Constants.IntakeConstants.GAINS.kI,
            Constants.IntakeConstants.GAINS.kD
        )

    def updateInputs(self, inputs: IntakeIO.IntakeIOInputs) -> None:
        """Update inputs with simulated state."""
        # Simulate motor behavior (simple integration)
        # In a real simulation, you'd use a physics model here

        if (self._closedLoop):
            self._motorAppliedVolts = self._controller.calculate(self._intakeSim.getAngularPosition())
        else:
            self._controller.reset(self._intakeSim.getAngularPosition(), self._intakeSim.getAngularAcceleration())

        self.setMotorVoltage(self._motorAppliedVolts)
        self._intakeSim.setInputVoltage(math.max(-12.0, min(self._motorAppliedVolts(12.0))))
    

        # Update inputs
        inputs.motorConnected = True
        inputs.motorPosition = self._motorPosition
        inputs.motorVelocity = self._motorVelocity
        inputs.motorAppliedVolts = self._motorAppliedVolts
        inputs.motorCurrent = abs(self._motorAppliedVolts / 12.0) * 40.0  # Rough current estimate
        inputs.motorTemperature = 25.0  # Room temperature

    def setOpenLoop(self, output):
        self._closedLoop = False
        self._motorAppliedVolts = output
    
    def setPosition(self, position):
        self._closedLoop = True
        self._controller.setSetpoint(rotationsToRadians(position))

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage (simulated)."""
        self._motorAppliedVolts = max(-12.0, min(12.0, voltage))
        # Simple velocity model: voltage -> velocity (with some damping)
        self._motorVelocity = self._motorAppliedVolts * 10.0  # Adjust multiplier as needed

