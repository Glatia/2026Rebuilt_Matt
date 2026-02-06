# Subsystem Conventions: `__init__.py` and `io.py`

This document describes how to implement each subsystem’s public API in `__init__.py` and its hardware/simulation abstraction in `io.py`. Use **hood** and **turret** as reference implementations.

---

## 1. `io.py` — Hardware/Simulation Abstraction

The `io.py` file defines the **IO layer**: an abstract interface and concrete implementations for real hardware and simulation. The subsystem depends only on the abstract interface, so the same logic runs on robot and in sim.

### 1.1 Abstract base class: `*IO(ABC)`

- **Name**: `{SubsystemName}IO` (e.g. `HoodIO`, `TurretIO`).
- **Role**: Contract for “what this subsystem can read and write,” shared by real and sim.

**Inner inputs dataclass**

- Define a **nested** `@dataclass` inside the IO class: `*IOInputs` (e.g. `HoodIO.HoodIOInputs`).
- Decorate with `@autolog` (from pykit) so it can be logged.
- Fields should use typed units where applicable: `radians`, `volts`, `amperes`, `celsius`, `radians_per_second`, etc. (from `wpimath.units` or phoenix6).
- Include at least:
  - **Connection**: e.g. `hood_connected: bool = False`
  - **State**: position, velocity (if relevant), applied volts, current, temperature
  - **Setpoint** (if closed-loop): e.g. `hood_setpoint: radians = 0.0`
- Use a consistent prefix for the subsystem (e.g. `hood_*`, `turret_*`) to avoid collisions in logging.

**Abstract methods**

- `update_inputs(self, inputs: *IOInputs) -> None`  
  Fills `inputs` with current hardware/simulation state. Called every period from the subsystem’s `periodic()`.
- One or more **output** methods that define how the subsystem commands the mechanism, e.g.:
  - `set_position(self, ...) -> None`
  - `set_voltage(self, volts: float) -> None`  
  Signatures and units (radians, Rotation2d, etc.) are chosen per subsystem.

Abstract methods in the base class should have docstrings and may use `pass` in the body.

### 1.2 Real hardware implementation: `*IOTalonFX(*IO)`

- **Name**: `{SubsystemName}IOTalonFX` (e.g. `HoodIOTalonFX`, `TurretIOTalonFX`).
- **Role**: Talks to real TalonFX (and any other hardware) on the robot.

**`__init__`**

- Take at least the CAN ID(s) and any other ports (e.g. PWM for servos). Optionally accept a pre-built config object (e.g. climber).
- Construct `TalonFX(motor_id, "rio")` and store in a `Final` field.
- Build a `TalonFXConfiguration`:
  - Set `slot0` (or appropriate slot) from `Constants.{Subsystem}Constants.GAINS`.
  - Set `feedback.sensor_to_mechanism_ratio` from `Constants.{Subsystem}Constants.GEAR_RATIO`.
  - Set `motor_output.neutral_mode` (e.g. `NeutralModeValue.BRAKE`) and `motor_output.inverted` as needed.
- Apply config with `tryUntilOk(5, lambda: self.*_motor.configurator.apply(motor_config, 0.25))`.
- Cache status signals: `get_position()`, `get_velocity()`, `get_motor_voltage()`, `get_stator_current()`, `get_device_temp()`, `get_closed_loop_reference()` (if used).
- Call `BaseStatusSignal.set_update_frequency_for_all(50, ...)` with all cached signals, then `*_motor.optimize_bus_utilization()`.
- Create control requests (e.g. `PositionVoltage(0)`) and store as instance attributes for use in output methods.

**`update_inputs`**

- Call `BaseStatusSignal.refresh_all(...)` with the cached signals.
- Set `inputs.*_connected = motor_status.is_ok()`.
- Copy each signal’s value into the corresponding `inputs.*_*` field (e.g. `value_as_double` for Phoenix 6).

**Output methods**

- Implement each abstract output method by building the appropriate control request and calling `*_motor.set_control(...)` (and/or setting other hardware). For position control, convert from mechanism units (e.g. radians) to rotations if required by the API.

### 1.3 Simulation implementation: `*IOSim(*IO)`

- **Name**: `{SubsystemName}IOSim` (e.g. `HoodIOSim`, `TurretIOSim`).
- **Role**: Simulates the mechanism so code can run without hardware.

**`__init__`**

- Create a `DCMotorSim` using `LinearSystemId.DCMotorSystem(motor, moi, gear_ratio)` and the same motor type/gearing as in constants. Use something like `DCMotor.krakenX44FOC(1)` or `DCMotor.krakenX60(1)` to match the real motor.
- Add a `PIDController` using `Constants.{Subsystem}Constants.GAINS` if the subsystem uses closed-loop position control.
- Track `applied_volts` and a `closed_loop` (or similar) flag so you can switch between open- and closed-loop in sim.

**`update_inputs`**

- If closed-loop: compute voltage with `controller.calculate(current_position)` and set setpoint from the last `set_position` (or equivalent) call. If open-loop: reset controller and use the last set voltage.
- Clamp voltage: `max(-12.0, min(self.applied_volts, 12.0))`, feed to `*_sim.setInputVoltage(...)`, then `*_sim.update(0.02)`.
- Fill all `inputs.*_*` fields from the sim (position, velocity, current draw, etc.). Set `inputs.*_connected = True`. Temperature can be a constant (e.g. 25.0) if not modeled.

**Output methods**

- For position: set `closed_loop = True` and `controller.setSetpoint(...)` in the units used by the subsystem (e.g. radians).
- If you support open-loop: provide something like `set_open_loop(self, output: volts)` that sets `closed_loop = False` and `applied_volts = output`.

### 1.4 Shared conventions in `io.py`

- Use `constants.Constants` and `util.tryUntilOk`; avoid magic numbers for IDs, gear ratios, and gains.
- Prefer `Final` for motors and cached signals in the TalonFX implementation.
- Keep units consistent: use `wpimath.units` / phoenix6 units in inputs and in public method signatures so the subsystem and IO layer agree (e.g. radians vs rotations).
- Add a `# pylint: disable=too-many-instance-attributes` only if the TalonFX class genuinely has many attributes and the team accepts it.

---

## 2. `__init__.py` — Subsystem Class and Public API

The `__init__.py` for a subsystem either defines the subsystem class in this file (like hood and turret) or loads it from a separate `*-subsystem.py` and re-exports it (like climber). In both cases, the file is responsible for the **public API** of the subsystem package.

### 2.1 What lives in `__init__.py`

- **Subsystem class**: Either defined here or imported from a sibling module (e.g. `climber-subsystem.py`).
- **Exports**: The subsystem class and the IO types that the rest of the codebase needs to construct the subsystem and choose real vs sim IO:
  - `*IO` (abstract base)
  - `*IOTalonFX`
  - `*IOSim`
  - Optionally `*IOInputs` if other code needs to reference the type; often it’s accessed as `*IO.*IOInputs` and doesn’t need a separate export.

### 2.2 Subsystem class design (in `__init__.py` or `*-subsystem.py`)

- **Base**: Subclass `Subsystem` from `subsystems` (or `StateSubsystem` if the subsystem is state-based).
- **Constructor**: Accept at least:
  - `io: *IO` — the abstract IO implementation (real or sim).
  - Any suppliers or shared state the subsystem needs (e.g. `robot_pose_supplier: Callable[[], Pose2d]`).
- **State**:
  - Store `self._io` (or `self.io`) and type it as the abstract `*IO`.
  - Create a single `*IOInputs` instance, e.g. `self._inputs = *IO.*IOInputs()` (or `self.inputs`).
  - Add alerts for hardware health, e.g. `Alert("... motor is disconnected.", Alert.AlertType.kError)`, and any debouncers or other helpers.
- **`periodic()`**:
  - Call `self._io.update_inputs(self._inputs)`.
  - Log inputs, e.g. `Logger.processInputs("SubsystemName", self._inputs)`.
  - Update alerts (e.g. disconnect) from `inputs.*_connected`.
  - Run subsystem logic (e.g. distance/angle calculations, goal-based aiming) and call the IO output methods (e.g. `self._io.set_position(...)`). All actual hardware or sim interaction goes through the IO interface.

### 2.3 Export list

- At the bottom of `__init__.py`, set `__all__` to the list of names that other modules should import from this package, e.g.:

  ```python
  __all__ = ["HoodIO", "HoodIOTalonFX", "HoodIOSim", "HoodSubsystem"]
  ```

  or with the subsystem loaded from another file:

  ```python
  __all__ = ["ClimberIO", "ClimberIOTalonFX", "ClimberIOSim", "ClimberSubsystem"]
  ```

### 2.4 Loading subsystem from a separate file (optional pattern)

If the subsystem class lives in `*-subsystem.py` (e.g. `climber-subsystem.py`) because of a hyphenated filename:

- Use `importlib.util.spec_from_file_location` and `module_from_spec` to load that module.
- Assign the module into `sys.modules` under a stable name, then `spec.loader.exec_module(...)`.
- Import the subsystem class from that module and re-export it in `__init__.py` so that `from subsystems.climber import ClimberSubsystem` works.

If the subsystem class lives in `__init__.py` (hood, turret), no importlib is needed; just define the class and list it in `__all__`.

### 2.5 Imports in `__init__.py`

- Import the base `Subsystem` (or `StateSubsystem`) from `subsystems`.
- Import the IO types from the local `io` module: `from subsystems.{subsystem}.io import *IO, *IOTalonFX, *IOSim`.
- Import constants, Logger, Alert, and any WPILib/PathPlanner types used by the subsystem logic. Keep `io.py` focused on hardware/sim; higher-level logic (e.g. field flipping, goals) stays in the subsystem class.

---

## 3. Quick checklist

**io.py**

- [ ] Abstract `*IO(ABC)` with inner `@autolog` `@dataclass` `*IOInputs` (connection + state + setpoint as needed).
- [ ] Abstract `update_inputs(inputs)` and output method(s).
- [ ] `*IOTalonFX`: config apply with `tryUntilOk`, status signals at 50 Hz, `optimize_bus_utilization`, implement `update_inputs` and outputs.
- [ ] `*IOSim`: `DCMotorSim` + PID (if closed-loop), voltage clamp, `update(0.02)`, implement `update_inputs` and outputs.

**__init__.py**

- [ ] Subsystem class takes `io: *IO` and any suppliers; holds `_inputs` and calls `_io.update_inputs(_inputs)` in `periodic()`.
- [ ] `periodic()` logs inputs, updates alerts, then runs logic and calls IO output methods.
- [ ] `__all__` lists `*IO`, `*IOTalonFX`, `*IOSim`, and the subsystem class.
- [ ] If the class is in a separate `*-subsystem.py`, load it with importlib and re-export the class.

Using these conventions keeps each subsystem testable in simulation, swappable between real and sim IO, and consistent with the rest of the project. Refer to `subsystems/hood/` and `subsystems/turret/` for concrete examples.
