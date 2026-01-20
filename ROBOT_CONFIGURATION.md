# Robot Configuration Guide

This project supports running on multiple robots (Larry - test robot, and Comp - competition robot) with different hardware configurations.

## How It Works

The system automatically detects which robot is running and selects the appropriate hardware configuration:

1. **Robot Detection** (`robot_config.py`):
   - Detects robot identity via MAC address, hostname, or environment variable
   - Sets `currentRobot` to `Robot.LARRY` or `Robot.COMP`

2. **Hardware Configuration** (`constants.py`):
   - Hardware constants (CAN IDs, ports, etc.) are initialized based on detected robot
   - Each robot can have different values for the same constant

3. **Usage**:
   - Access constants normally: `Constants.CanIDs.CLIMB_TALON`
   - The correct value for the current robot is automatically used

## Setting Up Robot Detection

### Method 1: MAC Address (Recommended)

Each RoboRIO has a unique MAC address. To find it:

```bash
# SSH into the RoboRIO
ssh admin@roborio-XXXX-frc.local

# Get the MAC address
cat /sys/class/net/eth0/address
```

Then update `robot_config.py`:

```python
LARRY_MAC_ADDRESSES = [
    "00:80:2f:XX:XX:XX",  # Replace with Larry's actual MAC
]
COMP_MAC_ADDRESSES = [
    "00:80:2f:YY:YY:YY",  # Replace with Comp's actual MAC
]
```

### Method 2: Hostname

Set different hostnames on each robot, then detection will work automatically if the hostname contains "larry" or "comp".

### Method 3: Environment Variable (For Testing)

Set the `ROBOT_NAME` environment variable:

```bash
# On Larry
export ROBOT_NAME=LARRY

# On Comp
export ROBOT_NAME=COMP
```

## Adding New Hardware Constants

To add robot-specific constants:

1. **Add to `Constants` class** in `constants.py`:

```python
class Constants:
    class NewSubsystemConstants:
        """New subsystem constants."""
        pass
```

2. **Initialize in `_init_hardware_configs()`**:

```python
def _init_hardware_configs():
    # ... existing code ...
    
    # New subsystem constants
    if currentRobot == Robot.LARRY:
        Constants.NewSubsystemConstants.MOTOR_ID = 20
        Constants.NewSubsystemConstants.SOME_VALUE = 1.5
    else:  # COMP
        Constants.NewSubsystemConstants.MOTOR_ID = 25
        Constants.NewSubsystemConstants.SOME_VALUE = 2.0
```

3. **Use in your code**:

```python
from constants import Constants

motor_id = Constants.NewSubsystemConstants.MOTOR_ID
```

## Example: Using in RobotContainer

```python
from constants import Constants
from robot_config import currentRobot, Robot
from subsystems.climber.io import ClimberIOTalonFX

# The correct CAN ID is automatically selected based on currentRobot
climber_io = ClimberIOTalonFX(
    Constants.CanIDs.CLIMB_TALON,  # Different value for Larry vs Comp
    Constants.ClimberConstants.SERVO_PORT,
    motor_config
)
```

## Debugging

To see which robot was detected, add logging:

```python
from robot_config import currentRobot
from pykit.logger import Logger

Logger.recordMetadata("Robot", currentRobot.name)
```

Or print it:

```python
from robot_config import currentRobot
print(f"Running on: {currentRobot.name}")
```

## Best Practices

1. **Always use `Constants` classes** - Don't hardcode CAN IDs or ports
2. **Document differences** - Add comments explaining why values differ between robots
3. **Test both robots** - Make sure code works on both Larry and Comp
4. **Use meaningful defaults** - If detection fails, it defaults to COMP (change if needed)

## Optional Subsystems

Larry (test robot) may not have all subsystems. To handle this:

### Adding/Removing Subsystems

Edit `robot_config.py` and update the `has_subsystem()` function:

```python
def has_subsystem(subsystem_name: str) -> bool:
    LARRY_SUBSYSTEMS = {
        "drivetrain",  # Always present
        "vision",      # Always present
        "climber",     # Add if Larry has climber
        # "intake",    # Comment out if Larry doesn't have intake
    }
    
    COMP_SUBSYSTEMS = {
        "drivetrain",
        "vision",
        "climber",
        "intake",
        # Add all Comp subsystems
    }
    # ... rest of function
```

### Using Optional Subsystems

Subsystems are created conditionally in `robot_container.py`:

```python
# Subsystems are Optional[SubsystemType]
self._climber: Optional[ClimberSubsystem] = None

# Create only if available
if has_subsystem("climber"):
    self._climber = ClimberSubsystem(climber_io)
```

### Checking if Subsystem Exists

Use helper methods or check for None:

```python
# In RobotContainer
if container.has_climber():
    climber = container.get_climber()
    climber.set_desired_state(...)

# Or check directly
if container._climber is not None:
    container._climber.set_desired_state(...)
```

### Superstructure with Optional Subsystems

The `Superstructure` class accepts optional subsystems:

```python
superstructure = Superstructure(
    drivetrain=container._drivetrain,
    vision=container._vision,
    climber=container._climber,  # Can be None
    intake=container._intake      # Can be None
)
```

Always check for None before using subsystems in `Superstructure.periodic()`:

```python
if self.climber is not None:
    # Use climber safely
    self.climber.set_desired_state(...)
```

## Troubleshooting

**Problem**: Wrong hardware configuration is being used

**Solution**: 
- Check MAC addresses in `robot_config.py`
- Verify hostname contains "larry" or "comp"
- Set `ROBOT_NAME` environment variable as fallback
- Check the default robot in `detect_robot()` function

**Problem**: Constants are not initialized

**Solution**: 
- Make sure `_init_hardware_configs()` is called at module load
- Check that `robot_config.py` is imported before `constants.py` uses `currentRobot`

**Problem**: Subsystem creation fails on Larry

**Solution**:
- Check that subsystem is listed in `LARRY_SUBSYSTEMS` in `has_subsystem()`
- Verify CAN IDs are correct for Larry in `constants.py`
- Check logs for "Subsystem not available on this robot" message
