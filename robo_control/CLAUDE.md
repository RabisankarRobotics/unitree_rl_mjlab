# CLAUDE.md - robo_control

Real-time bipedal robot control framework built on ROS 2 and C++17. Controls 12 servo motors (6 per leg) via EtherCAT or CAN bus, with ML policy inference, IMU sensing, joystick input, and WebRTC streaming.

## Repository Structure

```
robo_control/
├── robo_msgs/          # ROS 2 message/service definitions (.msg, .srv)
├── common/             # Header-only shared utilities (logging, threading, config, buffers)
├── transport/          # Header-only ROS 2 pub/sub abstraction layer
├── hardware/           # Low-level device SDKs
│   ├── ethercat_sdk/   #   EtherCAT motor control (uses SOEM)
│   ├── can_sdk/        #   CAN bus motor control
│   ├── imu_sdk/        #   BNO08x IMU driver
│   └── joystick_sdk/   #   Gamepad input
├── services/           # Microservices (each is a ROS 2 node)
│   ├── control_service/    # Main motor control loop (EtherCAT/CAN backends)
│   ├── policy_service/     # Neural network inference (MNN)
│   ├── imu_service/        # IMU data publisher
│   ├── joystick_service/   # Gamepad input publisher
│   ├── mode_service/       # Robot state machine
│   ├── media_service/      # WebRTC video/audio (GStreamer)
│   └── service_manager/    # Lifecycle orchestrator
├── config/             # YAML configs (hardware, services, per-service settings)
├── launch/             # ROS 2 launch files (robo.launch.py)
├── scripts/            # install-deps.sh, CAN setup, systemd sync
└── systemd/            # SystemD service file for auto-start
```

## Build System

**Colcon + CMake** (standard ROS 2 toolchain). All packages use ament_cmake.

### Building

```bash
# Source ROS 2 environment first
source /opt/ros/<distro>/setup.bash

# Build all packages
colcon build

# Build a specific package
colcon build --packages-select control_service

# Source the workspace overlay
source install/setup.bash
```

Colcon defaults (`.colcon/defaults.yaml`) enable `CMAKE_EXPORT_COMPILE_COMMANDS=ON` for IDE support.

### Installing Dependencies

```bash
# Install all dependencies
./scripts/install-deps.sh

# Install specific dependency
./scripts/install-deps.sh --mnn
./scripts/install-deps.sh --gstreamer
```

### Running

```bash
# Launch all services
ros2 launch launch/robo.launch.py

# Run a single service
ros2 run control_service control_service --ros-args -p config_path:=config/yaml/hardware.yaml
```

## Key Conventions

### C++ Style
- **Standard:** C++17
- **Compiler:** GCC 11 (clangd configured via `.clangd`)
- **Header-only libraries:** `common/` and `transport/` are header-only
- **Namespaces:** Match package name (`common::`, `transport::`, `ethercat_sdk::`, etc.)
- **Include guards:** `#pragma once`

### Architecture Patterns
- **Microservices via ROS 2 nodes:** Each service is an independent process communicating over DDS topics
- **Lock-free triple buffering:** `common::TripleBuffer` for real-time SPSC thread communication
- **Singleton NodeContext:** `transport::NodeContext` provides a shared ROS 2 node + MultiThreadedExecutor
- **Real-time ThreadLoop:** `common::ThreadLoop` with SCHED_FIFO/RR, CPU affinity, cycle-time monitoring
- **Service entry point pattern:** Services use `transport::service_main<T>()` as their main() boilerplate

### ROS 2 Topics
| Topic | Message Type | Publisher | Subscriber |
|---|---|---|---|
| `robo/joint_command` | JointCommand | policy_service | control_service |
| `robo/joint_state` | JointState | control_service | policy_service |
| `robo/imu_data` | IMUData | imu_service | policy_service |
| `robo/joystick` | Joystick | joystick_service | mode_service |
| `robo/robot_state` | RobotState | mode_service | policy_service, control_service |

### Configuration
- All config files live in `config/yaml/`
- `hardware.yaml` - Actuator definitions (EtherCAT/CAN interfaces, joint mappings, zero offsets)
- `services.yaml` - Service launcher config (package names, executables, params)
- Per-service YAML files for service-specific settings
- Config loading uses `common::config_loader.hpp` (wraps yaml-cpp)

### Adding a New Service
1. Create directory under `services/<name>/` with `src/`, `include/`, `CMakeLists.txt`, `package.xml`
2. Implement the service class, use `transport::service_main<T>()` for the entry point
3. Use `transport::Publisher<T>` / `transport::Subscriber<T>` for topic communication
4. Add config YAML to `config/yaml/`
5. Register in `config/yaml/services.yaml` for launch integration

### Adding a New Message
1. Add `.msg` or `.srv` file to `robo_msgs/msg/` or `robo_msgs/srv/`
2. Register in `robo_msgs/CMakeLists.txt` under `rosidl_generate_interfaces`
3. Add type alias in `transport/include/transport/transport.hpp`
4. Add topic name constant in `transport/include/transport/types.hpp`

## Dependencies

**Core:** ROS 2 (Humble/Jazzy), CMake, yaml-cpp, spdlog, Threads (POSIX)
**Hardware:** SOEM (EtherCAT), SocketCAN
**ML:** MNN (Alibaba lightweight inference)
**Media:** GStreamer 1.24, gst-plugins-rs (WebRTC), peel (C++ GObject bindings)

## Git

- Do not add `Co-Authored-By` lines to commit messages

## Notes

- The robot runs on an OrangePi board (ARM) with real-time Linux; development happens on x86
- `config/yaml` and `config/models` are gitignored - these contain deployment-specific configs and model files
- Services require root for real-time thread scheduling (SCHED_FIFO) and raw hardware access
- No test suite currently exists; hardware examples in `hardware/*/examples/` serve as manual integration tests
