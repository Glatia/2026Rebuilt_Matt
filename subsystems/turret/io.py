from abc import ABC
from dataclasses import dataclass, field
from typing import Final

from phoenix6 import BaseStatusSignal
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs, HardwareLimitSwitchConfigs, ProximityParamsConfigs, CurrentLimitsConfigs
from phoenix6.controls import VoltageOut, PositionVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue, InvertedValue
from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, volts, amperes, celsius, degrees, rotationsToRadians
from wpilib.simulation import DCMotorSim, EncoderSim
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.geometry import Rotation2d
from wpilib import Encoder, PWMTalonFX, RobotController
from wpimath.controller import PIDController
from wpimath.trajectory import TrapezoidProfile

from constants import Constants
from util import tryUntilOk


class TurretIO(ABC):
    """
    Abstract base class for Turret IO implementations.
    Provides the interface for both real hardware and simulation.
    """

    @autolog
    @dataclass
    class TurretIOInputs:
        """Inputs from the Turret hardware/simulation."""
        # Motor status
        turret_connected: bool = False
        turret_position: radians = 0.0
        turret_velocity: radians_per_second = 0.0
        turret_applied_volts: volts = 0.0
        turret_current: amperes = 0.0
        turret_temperature: celsius = 0.0
        turret_setpoint: radians = 0.0


    def updateInputs(self, inputs: TurretIOInputs) -> None:
        """Update the inputs with current hardware/simulation state."""
        pass

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage."""
        pass


class TurretIOTalonFX(TurretIO):
    
    def __init__(self, motor_id: int) -> None:
        self.turret_motor: Final[TalonFX] = TalonFX(motor_id, "rio")

        motor_config = TalonFXConfiguration()
        motor_config.slot0 = Constants.TurretConstants.GAINS
        motor_config.feedback.sensor_to_mechanism_ratio = Constants.TurretConstants.GEAR_RATIO
        motor_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
        motor_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

        tryUntilOk(5, lambda: self.turret_motor.configurator.apply(motor_config, 0.25))

        self.position = self.turret_motor.get_position()
        self.velocity = self.turret_motor.get_velocity()
        self.applied_volts = self.turret_motor.get_motor_voltage()
        self.current = self.turret_motor.get_stator_current()
        self.temperature = self.turret_motor.get_device_temp()
        self.setpoint = self.turret_motor.get_closed_loop_reference()

        BaseStatusSignal.set_update_frequency_for_all(
            50,
            self.position,
            self.velocity,
            self.applied_volts,
            self.current,
            self.temperature,
            self.setpoint
        )

        self.turret_motor.optimize_bus_utilization()

        self.position_request = PositionVoltage(0)

    def update_inputs(self, inputs: TurretIO.TurretIOInputs):
        motor_status = BaseStatusSignal.refresh_all(
        self.position,
        self.velocity,
        self.applied_volts,
        self.current,
        self.temperature,
        self.setpoint
    )
        
        inputs.turret_connected = motor_status.is_ok()
        inputs.turret_position = self.position.value_as_double
        inputs.turret_velocity = self.velocity.value_as_double
        inputs.turret_applied_volts = self.applied_volts.value_as_double
        inputs.turret_current = self.current.value_as_double
        inputs.turret_temperature = self.temperature.value_as_double
        inputs.turret_setpoint = self.setpoint.value_as_double

    def set_position(self, rotation: Rotation2d) -> None:
        """Set the position."""
        self.turret_motor.set_control(self.position_request)



class TurretIOSim(TurretIO):
    """
    Simulation implementation for testing without hardware.
    """

    def __init__(self) -> None:
        """Initialize the simulation IO."""
        self.motor = DCMotor.krakenX60(1)
        self.turretSim = DCMotorSim(LinearSystemId.DCMotorSystem(self.motor, .455, Constants.TurretConstants.GEAR_RATIO), self.motor) # MOI is a placeholder
        self.closed_loop = False

        self._motorPosition: float = 0.0
        self._motorVelocity: float = 0.0
        self.AppliedVolts: float = 0.0

        self.controller = PIDController(
            Constants.TurretConstants.GAINS.k_p,
            Constants.TurretConstants.GAINS.k_i,
            Constants.TurretConstants.GAINS.k_d,
            ) 

    def updateInputs(self, inputs: TurretIO.TurretIOInputs) -> None:
        """Update inputs with simulated state."""
        # Simulate motor behavior (simple integration)
        # In a real simulation, you'd use a physics model here
        dt = 0.02  # 20ms periodic

        if self.closed_loop:
            self.AppliedVolts = self.controller.calculate(self.turretSim.getAngularPosition())
        else:
            self.controller.reset()

        self.setMotorVoltage(self.AppliedVolts)
        self.turretSim.update(dt)

        inputs.motorConnected = True
        inputs.motorPosition = self.turretSim.getAngularPosition()
        inputs.motorVelocity = self.turretSim.getAngularAcceleration()
        inputs.motorAppliedVolts = self._motorAppliedVolts
        inputs.motorCurrent = abs(self.turretSim.getCurrentDraw())
        inputs.motorTemperature = 25.0  # Room temperature


    def setOpenLoop(self, output):
        self.closed_loop = False
        self.AppliedVolts = output

    def setPosition(self, position):
        self.closed_loop = True
        self.controller.getSetpoint(rotationsToRadians(position))


        """
        # Update inputs
        inputs.motorConnected = True
        inputs.motorPosition = self._motorPosition
        inputs.motorVelocity = self._motorVelocity
        inputs.motorAppliedVolts = self._motorAppliedVolts
        inputs.motorCurrent = abs(self._motorAppliedVolts / 12.0) * 40.0  # Rough current estimate
        inputs.motorTemperature = 25.0  # Room temperature

        self.inputVoltage = self.motor.get() * RobotController.getBatteryVoltage()
        self.motor_sim.setInputVoltage(self.inputVoltage)

        self.motor_sim.update(dt)  # 20ms periodic
        """

    def setMotorVoltage(self, voltage: volts) -> None:
        """Set the motor output voltage (simulated)."""
        self._motorAppliedVolts = max(-12.0, min(12.0, voltage))
        # Simple velocity model: voltage -> velocity (with some damping)
        self._motorVelocity = self._motorAppliedVolts * 10.0  # Adjust multiplier as needed

    