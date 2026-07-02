# Implementation History

This document tracks the sequence of architectural changes, new features, and bug fixes implemented in the Skyflock project. As new implementation plans are created, they will be appended to this log.

---

## 1. Swarm Orchestrator & Global Coordinate Alignment
**Date Implemented:** 2026-06-08
**Related To:** Swarm Formation Logic

### The Need
We had a functional low-level `swarm_controller.py` and `swarm_manager.py` capable of controlling 3 independent drones via MAVSDK. However, we lacked a high-level "brain" to coordinate multi-drone shapes (like a V-formation or a straight line) concurrently. Furthermore, because Gazebo spawns each drone at a different physical offset, commanding them to the same local NED coordinates caused them to cross paths and collide.

### The Solution
1. **`swarm_orchestrator.py` Node**: Created a dedicated ROS 2 node that subscribes to a `/swarm/command` topic. It acts as a client to the 3 existing `/drone_N/mission_command` services.
2. **Concurrent Execution**: The orchestrator triggers `GOTO`, `TAKEOFF`, and `START_OFFBOARD` commands asynchronously so all drones move simultaneously.
3. **Global Coordinate Alignment**: Added a `spawn_offsets` dictionary to the orchestrator. By subtracting each drone's physical Gazebo spawn location from the target global coordinate, all drones now operate in a single, shared Global Coordinate System. This prevents path-crossing and collisions during tight formation flying.

---

## 2. Continuous Setpoint Streaming
**Date Implemented:** 2026-06-08
**Related To:** Handbook Section 18. Advanced Improvements (Setpoint streaming inside DigiRC)

### The Need
Previously, the `SwarmManager.goto_position` method sent a target coordinate via `set_position_ned` exactly **once**. 
While MAVSDK's C++ backend attempts to maintain the offboard connection by streaming the last known setpoint, any interruption or failure from the orchestrator would result in the drone dropping out of `OFFBOARD` mode and falling back to `HOLD` or `RTL` mode. This caused visual anomalies, such as drones drifting out of formation or dropping to weird altitudes during flight maneuvers.

### The Solution
We implemented an "opt-in continuous-GOTO mode" loop directly inside `swarm_manager.py`. This acts as a background task that safely isolates setpoint streaming from the main command execution flow.

### Implementation Details:
1. **Target Tracking**: `SwarmManager.add_drone` now tracks `target_position` and a `setpoint_task` (asyncio Task) for each drone.
2. **Background Streaming Task**: Created an asynchronous method `_setpoint_loop()` that loops infinitely at 10Hz (0.1s sleep). 
    - If `target_position` is `None` (e.g. right after offboard is triggered but before a shape command), it sends `VelocityNedYaw(0, 0, 0, 0)` so the drone holds a steady hover.
    - If `target_position` is set, it continuously streams the `PositionNedYaw` coordinate.
3. **Task Lifecycle**: 
    - The task is created and attached to the event loop inside `start_offboard()`.
    - The task is canceled inside `land_drone()` to safely release control back to the flight controller.
4. **Command Updates**: `goto_position()` was updated to no longer call the drone SDK directly. It simply updates the dictionary variable `self.drones[drone_id]["target_position"]`, and the background task instantly adjusts the stream.

This approach guarantees that the drone's flight controller (PX4) receives an uninterrupted >2Hz stream of offboard commands, resulting in a locked-in, rigid swarm formation.

---

## 3. Swarm Keyboard Controller
**Date Implemented:** 2026-06-09
**Related To:** Interactive Flight Control (WASD/QE)

### The Need
Instead of using manual terminal commands (`ros2 topic pub ...`) to control the swarm, we need an interactive way to command takeoff, offboard mode, formation shifts, and real-time flight vectors (forward, backward, left, right, up, down) for the swarm as a cohesive unit.

### The Solution
1. **`swarm_keyboard_controller.py` Node**: A new node that captures keyboard inputs in non-blocking raw mode. It maps keystrokes (WASD for translation, QE for altitude, Space for landing, etc.) and publishes them to `/swarm/command`.
2. **Dynamic Swarm Center Control**: Updated `swarm_orchestrator.py` to maintain internal state for its center position (`center_x, center_y, center_z`) and active formation (`V` or `LINE`). When directional commands are received, the center coordinates are shifted by 1.0 meter, and all drone positions are dynamically updated and dispatched.
3. **Build Tooling**: Registered the new script in `CMakeLists.txt` for integration into the standard `colcon build` flow.

---

## 4. Swarm Telemetry Logger & ROS 2 Bridge
**Date Implemented:** 2026-06-10
**Related To:** Handbook Section 19. Developer Exercises (Telemetry Logger)

### The Need
While the swarm successfully receives and executes `GOTO` commands, we lack visibility into where the drones actually are in space. To verify formation accuracy, record flight paths, and enable future collision avoidance logic, we must extract the local NED (North-East-Down) position data from PX4 and broadcast it onto the ROS 2 network.

### The Solution
1. **Extract Telemetry (`swarm_manager.py`)**: Added an asynchronous `_telemetry_loop()` to the MAVSDK manager. It streams `position_velocity_ned()` data from PX4 and passes it to any registered callbacks.
2. **Bridge to ROS 2 (`swarm_controller.py`)**: The swarm controller registers a callback with the manager. Whenever new positional data arrives, it packages it into standard `geometry_msgs/msg/Point` messages and publishes them to `/drone_N/telemetry`.
3. **Log the Data (`swarm_telemetry_logger.py`)**: Created a new subscriber node that listens to all telemetry topics. It continuously writes the 3D coordinates (X, Y, Z) and timestamp to a `swarm_flight_log.csv` file for external graphing and analysis.

---

## 5. Swarm Battery Failsafe
**Date Implemented:** 2026-06-10
**Related To:** Handbook Section 19. Developer Exercises (Battery Failsafe)

### The Need
A purely safety-oriented feature. If a drone's battery dies mid-flight, it will drop out of the sky. We need a system that independently monitors the battery level of each drone and forcefully triggers a safe Return-To-Launch (RTL) override if it dips below a critical threshold (20%).

### The Solution
1. **Battery Monitor Task**: Added an asynchronous `_battery_monitor_loop()` task inside `swarm_manager.py` that listens to MAVSDK's `telemetry.battery()` stream. This task is spawned for every drone upon connection.
2. **The Override Logic**: If `battery.remaining_percent < 0.20`, the script does three things:
   - **Cancels Offboard Streaming**: It forcefully cancels the drone's active `setpoint_task` (the 10Hz position streamer). This is critical, as continuing to stream offboard coordinates would conflict with the RTL flight mode.
   - **Triggers RTL**: It sends `drone.action.return_to_launch()`.
   - **Breaks**: It exits the monitoring loop so it doesn't spam the RTL command repeatedly.

---

## 6. Swarm HOVER Command
**Date Implemented:** 2026-06-10
**Related To:** Handbook Section 19. Developer Exercises (Add a HOVER command)

### The Need
The user needed an "emergency brake" or "pause" button. If the swarm is flying toward a new formation target, the user must be able to hit a single key to force all drones to stop and hover exactly where they currently are.

### The Solution
1. **Manager Support (`swarm_manager.py`)**: Added an asynchronous `hover_drone()` method. Instead of reading the drone's exact current coordinate and commanding it to go there, we simply set `target_position = None`. Our previously built `_setpoint_loop` detects the `None` value and automatically falls back to streaming `VelocityNedYaw(0, 0, 0, 0)`, which forces PX4 into a smooth, instantaneous hover.
2. **Orchestrator Routing (`swarm_orchestrator.py`)**: Added a `hover_all()` method and mapped the string `"HOVER"` to execute it across all drones.
3. **Keyboard Binding (`swarm_keyboard_controller.py`)**: Mapped the `h` key to publish the `"HOVER"` command.

---

## 7. Swarm Yaw Control
**Date Implemented:** 2026-06-10
**Related To:** Handbook Section 19. Developer Exercises (Yaw Control)

### The Need
To allow dynamic heading changes without overriding the active position stream, we needed a dedicated RC-style channel. Previously, the swarm could fly forward or sideways while maintaining an absolute 0-degree North orientation, which is unnatural for a drone flying complex paths.

### The Solution
1. **New RC Channel (`swarm_controller.py`)**: We introduced independent subscriptions to `/drone_N/yaw_command` (using standard `Float32` messages). The callbacks dynamically feed new heading angles into the running swarm manager without disrupting the main command pipeline.
2. **Dynamic Setpoint Updates (`swarm_manager.py`)**: Added an `update_yaw()` method. Since `_setpoint_loop` streams `target_position` at 10Hz, `update_yaw()` intercepts the active `PositionNedYaw` object, updates its `yaw_deg` component, and immediately saves it back to the state dictionary. This results in the drone smoothly rotating in place or during flight.

---

## 8. Swarm Mission Scripting
**Date Implemented:** 2026-06-11
**Related To:** Autonomous Patterns and Scripting

### The Need
While manual WASD and formation commands are useful, true swarm behavior requires autonomous pattern execution without manual babysitting, yet with instant interruption for safety.

### The Solution
1. **Mission Thread (`swarm_orchestrator.py`)**: Added a `trigger_square_mission()` method that spawns a background thread to execute a 10x10 square flight path. This prevents blocking the ROS 2 event loop, allowing the node to still process telemetry and safety commands.
2. **Safety Aborts**: Added a `mission_active` flag. Any manual command (`h` for hover, `w` for forward, etc.) instantly sets `mission_active = False`. The background thread checks this flag continuously and aborts the mission the moment the user takes over.
3. **Trigger**: Added a new `'m'` keyboard binding in `swarm_keyboard_controller.py` to trigger the `SQUARE` mission.

---

## 9. Swarm Scaling to 5 Drones
**Date Implemented:** 2026-06-12
**Related To:** Handbook Section 19. Developer Exercises (Scale Test)

### The Need
To prove the architecture's modularity and verify that the 10 Hz telemetry network isn't bottlenecked, the system needs to support more drones. The final handbook exercise (Exercise 7) requires scaling the swarm from 3 to 5 drones.

### The Solution
1. **Parameterized Node Generation**: Refactored `swarm_controller.py`, `swarm_orchestrator.py`, and `swarm_telemetry_logger.py` to replace hardcoded ranges with `NUM_DRONES = 5` constants and dynamic loops. Service, publisher, and subscriber creation now automatically scales to any N.
2. **Launch Adjustments**: Updated `start_swarm.sh` to spawn PX4 SITL instances for Drones 4 and 5 with increasing X-axis offsets (`PX4_GZ_MODEL_POSE`) and incremental instance IDs (`-i 3`, `-i 4`).
3. **Formation Math Expansion**: Expanded the orchestrator's `form_v` and `form_line` methods to calculate positions for the outer wings (Drone 4 and 5), maintaining structural integrity as the swarm grows.

---

## 10. Code Quality & Dead Code Cleanup
**Date Implemented:** 2026-06-16
**Related To:** Handbook Section 20.1 (Remove dead code), Section 18 (Better async hygiene)

### The Need
The codebase contained 3 dead/demo files (`multi_drone.py`, `my_node.py`, `async_demo.py`), bare `print()` calls instead of structured logging, an unused `from requests import request` import, and a broken standalone `main()` function in `swarm_manager.py`. Additionally, `NUM_DRONES` was hardcoded in each file with inconsistent values.

### The Solution
1. **Dead Code Removal**: Deleted 3 demo files and removed `my_node.py` from `CMakeLists.txt`.
2. **Structured Logging**: `SwarmManager` now accepts a logger (ROS or stdlib) in its constructor, replacing all `print()` calls.
3. **Command Validation**: Added a `VALID_COMMANDS` set and `validate_position()` with clamping to safe ranges.
4. **Battery Monitor Fix**: Moved the health check from after the battery loop into `connect_drone()`.
5. **Environment Variables**: All nodes now read `NUM_DRONES` from `os.environ.get("NUM_DRONES", "3")`, and spawn offsets are generated dynamically from `SPAWN_SPACING_X`.

---

## 11. Code Modularization
**Date Implemented:** 2026-06-16
**Related To:** Handbook Section 20.1 (Split the one file into modules)

### The Need
`swarm_manager.py` (305 lines) handled 6+ responsibilities: connection, commands, telemetry, battery monitoring, setpoint streaming, and yaw updates. This made it hard to test and maintain.

### The Solution
Created a `manager/` Python package with focused sub-modules: `connection.py`, `commands.py`, `telemetry.py`, `setpoint.py`, and a composed `manager.py`. The public API is unchanged — `swarm_controller.py` only needed one import line changed.

---

## 12. HAL Abstraction Layer
**Date Implemented:** 2026-06-16
**Related To:** Handbook Section 18 (Full HAL adoption), Section 20.1 (Implement AbstractDroneHAL)

### The Need
Switching between simulation and real hardware required branching throughout the code. The handbook recommends an abstract HAL so sim/real is a clean swap.

### The Solution
Created a `hal/` package with `AbstractDroneHAL` (15-method ABC), `SitlHAL` (MAVSDK implementation for Gazebo simulation), and `PixhawkHAL` (stub for real hardware). When real Pixhawk drones are available, implementing `PixhawkHAL` enables flight without changing any other code.

---

## 13. QoS Tuning
**Date Implemented:** 2026-06-16
**Related To:** Handbook Section 18 (QoS tuning)

### The Need
All publishers and subscribers used the default `depth=10` QoS. For a busy network with 5+ drones, this wastes resources on high-rate telemetry and risks dropping critical commands.

### The Solution
Applied explicit QoS profiles: `BEST_EFFORT` for telemetry publishers/subscribers (high-rate, dropping is ok), `BEST_EFFORT depth=1` for yaw (only latest heading matters), and `RELIABLE` for command subscribers (commands must not be dropped).

---

## 14. Unit Tests
**Date Implemented:** 2026-06-16
**Related To:** Handbook Section 20.1 (Add unit tests for pure logic)

### The Need
Zero test coverage. The handbook recommends testing frame math, command validation, and spawn offset logic.

### The Solution
Created 23 pure-logic unit tests in `tests/` covering: `VALID_COMMANDS` membership, `_clamp()` utility, `validate_position()` edge cases, V-formation math (1/3/5 drones, symmetry, custom spacing), line formation, spawn offset generation, and GOTO with offset subtraction.

---

## 15. Altitude Synchronization and Flight Logging
**Date Implemented:** 2026-06-17
**Related To:** Multi-Drone SITL Accuracy & Telemetry

### The Need
During PX4 SITL spawning, multiple drones can establish their "Home Position" (`Z=0` reference) at slightly different absolute altitudes due to terrain settling and barometer drift. As a result, when told to fly `8m up` locally, they ended up at different visual and physical heights. Furthermore, telemetry logs were cluttering the root directory and needed to be organized.

### The Solution
1. **Dynamic Altitude Synchronization (`swarm_manager.py`)**: 
   - We now intercept the `telemetry.home()` absolute altitude during drone connection.
   - We designate Drone 1's home altitude as the `REFERENCE_ALTITUDE`.
   - We automatically calculate and apply a `z_offset` for all other drones, both for the initial `takeoff` altitude and all subsequent `GOTO` local NED setpoints. This ensures all drones perfectly match Drone 1's global elevation.
2. **Flight Logs Directory (`swarm_telemetry_logger.py`)**: Modified the logger to dynamically create a `flight_logs` directory and save all CSV recordings there, keeping the workspace clean.

## 16. Observability & Diagnostics
**Date Implemented:** 2026-06-17
**Related To:** Handbook Section 18. Advanced Improvements (Observability)

### The Need
To graduate from a prototype to a production-ready system, we needed real-time visibility into the health and performance of the drone swarm. Standard terminal prints are insufficient for monitoring continuous metrics like battery decay, telemetry latency, or GPS health across many nodes.

### The Solution
1. **JSON Structured Logging**: We introduced a `JsonLoggerWrapper` in `swarm_controller.py`. It overrides standard ROS 2 `info/warn/error` calls to emit JSON strings containing the timestamp, severity, and message, making log ingestion trivial for systems like ELK or Datadog.
2. **Prometheus Metrics Engine**: We integrated the `prometheus_client` library to spin up an HTTP metrics server on port 8000. It tracks:
   - `drone_altitude_m` (Gauge): Real-time Z-axis position.
   - `drone_battery_percent` (Gauge): Real-time battery status.
   - `swarm_commands_total` (Counter): Tracking executed commands per drone.
3. **ROS 2 `/diagnostics` Integration**: We added the `diagnostic_msgs` dependency and created a publisher for `/diagnostics`. The system now leverages MAVSDK's EKF health states (`is_global_position_ok`, `is_home_position_ok`) and maps them to standard `DiagnosticStatus.OK` and `ERROR` flags, seamlessly bridging the PX4 layer to the standard ROS 2 diagnostics ecosystem.

---

## 17. Independent Drone Control & Detachment
**Date Implemented:** 2026-07-02
**Related To:** Autonomous Formations & Human Override

### The Need
During a swarm mission (like a revolution or formation flight), the operator might need to temporarily take manual control of a single drone (e.g., to inspect something) while the rest of the swarm continues its automated mission undisturbed. The operator then needs a way to command that drone to instantly rejoin the swarm formation.

### The Solution
1. **Selection & State Management**: Added `active_swarm` and `detached_drones` sets to the Orchestrator. Pressing keys `1` through `9` sets the `selected_target` and moves the drone into `detached_drones`. 
2. **Formation Gap Handling**: The formation and revolution algorithms (V, Line, Orbit) were updated to calculate slots for all drones but *skip sending commands* to detached drones. This perfectly maintains the structural integrity of the formation while leaving a physical "hole" where the detached drone used to be.
3. **Independent Movement**: When a specific drone is selected, manual commands (WASD, QE, etc.) are routed exclusively to that drone's cached target coordinates instead of shifting the entire swarm center.
4. **Instant Rejoin**: Pressing `p` (`REJOIN`) immediately moves the selected drone back into the `active_swarm` set. The Orchestrator automatically commands it to fly directly to its assigned slot in the ongoing formation or orbit.

---

## 18. Bugfix: Intermittent Takeoff Failure & Race Conditions
**Date Implemented:** 2026-07-02
**Related To:** Multi-Drone SITL Startup & `takeoff_drone` Sequence

### The Issue
During testing, commanding the swarm to `TAKEOFF` occasionally caused a drone to remain on the ground. Flight logs (`.ros/log/`) revealed that when the orchestrator dispatched `TAKEOFF` commands to all drones concurrently, all `MAVSDK` instances would wait for EKF (Estimator) health to converge. Since they finished initialization around the same time, all instances would attempt to execute `HOLD`, set altitude, and `ARM` at the exact same millisecond. This massive spike in concurrent RPC/socket requests and Gazebo physics updates caused race conditions in the PX4 SITL bridge, resulting in dropped commands or `COMMAND_DENIED` errors.

### The Solution
I introduced a **mathematical stagger delay** in `src/my_digirc/src/manager/commands.py`. Right after the `health OK` synchronization point, the sequence calculates a delay based on the drone ID:
`stagger_delay = (drone_id - 1) * 2.0`
This artificially desynchronizes the remainder of the sequence. Even if all drones become healthy simultaneously, Drone 1 arms immediately, Drone 2 waits 2.0s, and Drone 3 waits 4.0s. This decouples the heavy initialization load and ensures reliable arming/takeoff for every drone in the swarm.
