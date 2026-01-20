# Climber Subsystem IO Layer

This directory contains the PyKit IO layer implementation for the climber subsystem, following the pattern established by [PyKit](https://github.com/1757WestwoodRobotics/PyKit).

## Files

- **`io.py`** - Contains the IO layer abstract base class and implementations:
  - `ClimberIO` - Abstract base class defining the interface
  - `ClimberIOTalonFX` - Real hardware implementation using TalonFX and Servo
  - `ClimberIOSim` - Simulation implementation for testing without hardware

- **`example_refactored.py`** - Example showing how to refactor `ClimberSubsystem` to use the IO layer

- **`__init__.py`** - Package initialization file

## Usage Pattern

### 1. In `robot_container.py`

Instantiate the climber subsystem with the appropriate IO implementation based on the current mode:

```python
from subsystems.climber import ClimberSubsystem
from subsystems.climber.io import ClimberIOTalonFX, ClimberIOSim
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, MotorOutputConfigs, FeedbackConfigs

# In RobotContainer.__init__():
match Constants.currentMode:
    case Constants.Mode.REAL:
        # Create motor config
        motor_config = (TalonFXConfiguration()
            .with_slot0(Constants.ClimberConstants.GAINS)
            .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
            .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ClimberConstants.GEAR_RATIO))
        )
        
        # Create real hardware IO
        climber_io = ClimberIOTalonFX(
            Constants.CanIDs.CLIMB_TALON,
            Constants.ClimberConstants.SERVO_PORT,
            motor_config
        )
        
        # Create subsystem with real hardware IO
        self._climber = ClimberSubsystem(climber_io)
        
    case Constants.Mode.SIM:
        # Create subsystem with simulation IO
        self._climber = ClimberSubsystem(ClimberIOSim())
```

### 2. In `ClimberSubsystem`

The subsystem should:
- Accept an `ClimberIO` instance in `__init__`
- Store the IO instance and create an inputs object
- Call `updateInputs()` in `periodic()` and log with `Logger.processInputs()`
- Use IO methods (`setMotorVoltage()`, `setServoAngle()`) to control hardware
- Read from `_inputs` to get sensor data

See `example_refactored.py` for a complete example.

## Key Benefits

1. **Hardware Abstraction**: Easy to swap between real hardware and simulation
2. **Logging**: All inputs are automatically logged via PyKit's `@autolog` decorator
3. **Replay Support**: Logs can be replayed for debugging and analysis
4. **Testability**: Simulation implementation allows testing without hardware
5. **Consistency**: Follows the same pattern as other subsystems (Drive, Vision)

## Adding New States

To add new climber states:

1. Add the state to `SubsystemState` enum
2. Add the state configuration to `_state_configs` dictionary:
   ```python
   _state_configs: dict[SubsystemState, tuple[float, float]] = {
       SubsystemState.STOP: (0.0, Constants.ClimberConstants.SERVO_DISENGAGED_ANGLE),
       SubsystemState.CLIMB_IN: (Constants.ClimberConstants.VOLTAGE_INWARDS, Constants.ClimberConstants.SERVO_ENGAGED_ANGLE),
       # ... more states
   }
   ```

## Extending the IO Layer

If you need to add more functionality:

1. Add methods to the `ClimberIO` abstract base class
2. Implement the methods in both `ClimberIOTalonFX` and `ClimberIOSim`
3. Add any new inputs to `ClimberIOInputs` dataclass (they'll be automatically logged)

## Notes

- The simulation implementation (`ClimberIOSim`) uses a simple model. For more accurate simulation, consider integrating with WPILib's physics simulation.
- Motor position is tracked in rotations. Adjust the `get_position()` method if you need different units.
- The servo doesn't provide feedback, so the IO layer tracks the last set angle.
