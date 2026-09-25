# ethercat-sdk Python Bindings

Python bindings for the `ethercat_sdk` EtherCAT motor control library, built with [nanobind](https://github.com/wjakob/nanobind).

## Installation (end users)

The package ships as a pre-compiled wheel — no compiler, no system dev libs, no
colcon, no ROS required.

```bash
pip install ethercat_sdk-1.0.0-cp312-abi3-linux_x86_64.whl
```

The wheel targets CPython 3.12+ on Linux x86_64. The `libsoem` runtime
dependency is bundled inside the wheel. Actuator type definitions
(name → `rated_torque_nm`) are hardcoded in the SDK — to add a motor model,
edit the `getActuatorTypeTable()` entry in `hardware/ethercat_sdk/src/master.cpp`
and rebuild.

```python
import ethercat_sdk
master = ethercat_sdk.EtherCATMaster("eno1")
```

## Building the wheel (maintainers)

The build flow uses [uv](https://docs.astral.sh/uv/) for the wheel build and
`auditwheel` to bundle native dependencies into a self-contained, distributable
wheel.

### Build-host prerequisites

- Linux x86_64
- Python >= 3.12 and [uv](https://docs.astral.sh/uv/getting-started/installation/)
- A C++17 compiler (GCC 11+)
- System libraries: `libspdlog-dev` and SOEM
  (`/usr/local/lib/libsoem.so` from a `cmake --install` of OpenEtherCATsociety/SOEM)

`scikit-build-core` and `nanobind` are fetched automatically by uv into an
isolated build environment per [PEP 517][pep-517].

### Build

From the repo root:

```bash
hardware/ethercat_sdk_py/scripts/build-wheel.sh
```

This runs:

```bash
uv build --wheel --no-sources       # -> dist/ethercat_sdk-*.whl
uvx --from auditwheel auditwheel repair dist/*.whl -w wheelhouse/
```

The final wheel lands at `hardware/ethercat_sdk_py/wheelhouse/ethercat_sdk-*-manylinux_*.whl`.
Distribute that file and install it with `pip install`.

### Notes

- The wheel uses CPython's [stable ABI][abi3] (`cp312-abi3`), so a single
  artifact covers all Python 3.12+ interpreters.
- `aarch64` (OrangePi) wheels need to be built on an `aarch64` host — this
  script does not cross-compile.
- `ethercat_sdk` C++ sources are compiled directly into the extension module;
  the wheel does **not** depend on a separately-built `libethercat_sdk.so`.

[pep-517]: https://peps.python.org/pep-0517/
[abi3]: https://docs.python.org/3/c-api/stable.html

## Quick Start

```python
import ethercat_sdk

master = ethercat_sdk.EtherCATMaster("enp3s0")
master.config_actuator_types({1: "X8-120"})
master.set_operation_mode(ethercat_sdk.OperationMode.PVT)

if not master.init():
    raise RuntimeError("Failed to initialize EtherCAT master")

master.start()
master.enable_all()

# Read state
state = master.get_actuator_state(1)
print(f"pos={state.position:.4f} vel={state.velocity:.4f}")

# Send PVT command (velocity and torque default to 0.0)
master.set_pvt(1, position=0.0, kp=50.0, kd=2.0)

master.disable_all()
master.stop()
```

### Context Manager

The `EtherCATMaster` supports context manager usage for automatic cleanup:

```python
master = ethercat_sdk.EtherCATMaster("enp3s0")
master.config_actuator_types({1: "X8-120"})
master.set_operation_mode(ethercat_sdk.OperationMode.PVT)
master.init()

with master:  # calls start()
    master.enable_all()
    state = master.get_actuator_state(1)
    master.set_pvt(1, position=state.position, kp=50.0, kd=2.0)
# automatically calls disable_all() + stop()
```

### PVT Mode

```python
master.set_operation_mode(ethercat_sdk.OperationMode.PVT)
master.init()
master.start()
master.enable_all()

# Impedance control (velocity and torque default to 0.0)
master.set_pvt(1, position=0.5, kp=50.0, kd=2.0)

# With feed-forward velocity and torque
master.set_pvt(1, position=0.5, kp=50.0, kd=2.0, velocity=0.1, torque=0.2)
```

## API Reference

### Enums

**`OperationMode`** - Motor control mode (set before `init()`)

| Value | Description |
|-------|-------------|
| `CSP` | Cyclic Synchronous Position |
| `CSV` | Cyclic Synchronous Velocity |
| `CST` | Cyclic Synchronous Torque |
| `PVT` | Position-Velocity-Torque with KP/KD |

**`DriveState`** - CiA 402 drive state machine

`NotReadyToSwitchOn`, `SwitchOnDisabled`, `ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled`, `QuickStopActive`, `FaultReactionActive`, `Fault`

### Data Types

**`ActuatorState`** - Feedback from an actuator

| Field | Type | Description |
|-------|------|-------------|
| `position` | `float` | Position in radians |
| `velocity` | `float` | Velocity in rad/s |
| `torque` | `float` | Torque in Nm |
| `motor_temp` | `float` | Motor temperature in C (Standard mode) |
| `voltage` | `float` | Bus voltage in V (Standard mode) |
| `first_encoder` | `int` | Raw encoder value (PVT mode) |
| `error_code` | `int` | Drive error code |
| `drive_state` | `DriveState` | Current drive state |
| `mode` | `OperationMode` | Active operation mode |
| `enabled` | `bool` | Drive is enabled |
| `fault` | `bool` | Drive is in fault |
| `lost` | `bool` | Communication lost |

**`ActuatorType`** - Actuator type definition

| Field | Type | Description |
|-------|------|-------------|
| `rated_torque_nm` | `float` | Rated torque (used for CiA 402 per-mille torque scaling) |

**`BusConfig`** - Bus configuration

| Field | Type | Description |
|-------|------|-------------|
| `interface_name` | `str` | Network interface |
| `loop_hz` | `int` | Control loop frequency |
| `cycle_time_us` | `int` | Cycle time in microseconds |
| `actuators` | `list[ActuatorConfig]` | Actuator configurations |

**`CycleTimingSnapshot`** - Per-cycle timing diagnostics (nanosecond precision)

| Field | Type | Description |
|-------|------|-------------|
| `intended_wakeup_ns` | `int` | Requested wakeup time |
| `wakeup_ns` | `int` | Actual wakeup time |
| `after_receive_ns` | `int` | After receiving process data |
| `after_txpdo_read_ns` | `int` | After reading TxPDO feedback |
| `after_rxpdo_write_ns` | `int` | After writing RxPDO commands |
| `after_send_ns` | `int` | After sending process data |
| `after_dc_sync_ns` | `int` | After DC clock sync |
| `cycle_end_ns` | `int` | End of cycle |
| `dc_offset_ns` | `int` | DC PI controller offset |
| `wkc` | `int` | Working counter |
| `expected_wkc` | `int` | Expected working counter |
| `cycle_count` | `int` | Monotonic cycle counter |

### EtherCATMaster

```python
EtherCATMaster(interface_name: str, cycle_time_us: int = 1000)
```

**Configuration:**

| Method | Description |
|--------|-------------|
| `config_actuator_types(map: dict[int, str])` | Map bus IDs to actuator type names |
| `get_available_actuator_types() -> list[str]` | List registered actuator types |
| `has_actuator_type(type_name: str) -> bool` | Check if type is registered |
| `get_actuator_type(type_name: str) -> ActuatorType \| None` | Get type definition |
| `set_operation_mode(mode: OperationMode)` | Set mode (before `init()`) |
| `get_operation_mode() -> OperationMode` | Get current mode |

**Lifecycle:**

| Method | Description |
|--------|-------------|
| `init() -> bool` | Initialize SOEM, discover slaves |
| `start()` | Start real-time control loop |
| `stop()` | Stop control loop |

**Status:**

| Method | Description |
|--------|-------------|
| `get_slave_count() -> int` | Number of discovered slaves |
| `is_running() -> bool` | Control loop is active |
| `is_operational() -> bool` | Slaves are in OPERATIONAL state |
| `get_working_counter() -> int` | Current working counter |
| `get_expected_wkc() -> int` | Expected working counter |

**Actuator Control:**

| Method | Description |
|--------|-------------|
| `enable(actuator_id: int)` | Enable a single actuator |
| `disable(actuator_id: int)` | Disable a single actuator |
| `clear_fault(actuator_id: int)` | Clear fault on an actuator |
| `enable_all()` | Enable all actuators |
| `disable_all()` | Disable all actuators |

**Commands:**

| Method | Description |
|--------|-------------|
| `set_command(id, position, velocity, torque, kp=0, kd=0)` | Set all command fields |
| `set_position(id, position)` | Set target position (rad) |
| `set_velocity(id, velocity)` | Set target velocity (rad/s) |
| `set_torque(id, torque)` | Set target torque (Nm) |
| `set_max_torque(id, max_torque)` | Set max torque limit (Nm) |
| `set_pvt(id, position, kp, kd, velocity=0, torque=0)` | Set PVT with impedance gains |

**Feedback:**

| Method | Description |
|--------|-------------|
| `get_actuator_state(actuator_id: int) -> ActuatorState` | Get actuator feedback |
| `get_cycle_stats() -> CycleTimingSnapshot \| None` | Get cycle timing data |
| `get_config() -> BusConfig` | Get bus configuration |

### Exceptions

- **`InvalidActuatorTypeError`** (subclass of `RuntimeError`) - Raised when an unknown actuator type is configured.

## Running Tests

```bash
pip install "hardware/ethercat_sdk_py/[test]"
pytest hardware/ethercat_sdk_py/tests/
```

Tests are hardware-independent smoke tests that verify the module loads correctly.
