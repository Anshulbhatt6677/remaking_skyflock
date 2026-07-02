# digiRC — Developer Documentation

A multi-drone swarm control system built on **ROS 2 + PX4 + MAVSDK**, tested in Gazebo SITL.

---

## 1. Prerequisites

Before cloning, make sure you have these installed:

| Dependency | Version | Purpose |
|---|---|---|
| **Ubuntu** | 22.04+ | OS (ROS 2 Humble targets this) |
| **ROS 2 Humble** | — | Node communication framework |
| **PX4-Autopilot** | v1.14+ | Flight controller firmware (cloned to `~/PX4-Autopilot`) |
| **Gazebo Harmonic** | — | 3D physics simulator (installed with PX4) |
| **MAVSDK-Python** | `pip install mavsdk` | Python SDK to talk to PX4 |
| **Python 3.10+** | — | Runtime |
| **colcon** | — | ROS 2 build tool |

Optional:
- `pip install prometheus_client` — enables live metrics dashboard on port 8000

---

## 2. Setup

```bash
# 1. Clone the repo
git clone https://github.com/Anshulbhatt6677/remaking_skyflock.git
cd remaking_skyflock

# 2. Build the ROS 2 package
colcon build
source install/setup.bash

# 3. Make the startup script executable
chmod +x start_swarm.sh
```

---

## 3. Running the Swarm

```bash
./start_swarm.sh
```

This single script does everything:

| Step | Wait | What happens |
|---|---|---|
| Cleanup | 2s | Kills leftover PX4/Gazebo/ROS processes from previous runs |
| Drone 1 | 15s | Launches Gazebo world + first PX4 instance |
| Drone 2 | 10s | Spawns second PX4 at position (5,0,0) |
| Drone 3 | — | Spawns third PX4 at position (10,0,0) |
| EKF wait | 60s | Waits for all drones' navigation filters to lock GPS |
| ROS nodes | 6s | Starts controller, orchestrator, logger (2s apart) |
| Keyboard | — | Opens keyboard controller in foreground |

**Total boot: ~90 seconds.** Once you see the key-binding help text, you're ready.

### Flight Workflow

Press these keys **in order** for your first flight:

1. `t` — **Takeoff** (all drones arm and climb to 8m)
2. Wait ~10 seconds for drones to reach altitude
3. `o` — **Offboard** (switches to computer-controlled flight mode)
4. `v` — **V-Formation** (drones arrange into a V shape)
5. `w` — **Forward** (move swarm 1m forward, press repeatedly)
6. `h` — **Hover** (freeze in place)
7. `k` — **Land** (all drones descend and disarm)

### All Key Bindings

| Key | Command | Key | Command |
|---|---|---|---|
| `w/s/a/d` | Forward / Back / Left / Right | `v` | V-formation |
| `q/e` | Up / Down | `l` | Line-formation |
| `j/i` | Rotate left/right (±15°) | `t` | Takeoff |
| `c` | Toggle orbit revolution | `o` | Offboard mode |
| `r` | Toggle yaw rotation | `h` | Hover |
| `x/z` | Reverse revolution/rotation | `k`/Space | Land |
| `m` | Square mission (autonomous) | Ctrl+C | Quit |
| `1`-`9` | Select & Detach Drone N | `p` | Rejoin formation |
| `0` | Re-select Swarm (default) | | |

---

## 4. Architecture Overview

```
Keyboard Controller  →  publishes String to /swarm/command
        ↓
Swarm Orchestrator   →  does formation math, calls /drone_N/mission_command
        ↓
Swarm Controller     →  bridges ROS 2 to MAVSDK asyncio loop
  + SwarmManager     →  sends MAVSDK commands to PX4 over UDP
        ↓
PX4 SITL (Gazebo)    →  one instance per drone
```

A separate **Telemetry Logger** subscribes to `/drone_N/telemetry` and writes CSV files.

### Port Allocation (auto-calculated from drone ID)

| Drone | PX4 UDP | MAVSDK gRPC | Spawn X |
|---|---|---|---|
| 1 | 14540 | 50051 | 0 |
| 2 | 14541 | 50052 | 5 |
| 3 | 14542 | 50053 | 10 |
| N | 14540+(N-1) | 50050+N | (N-1)×5 |

---

## 5. Code Walkthrough — Module by Module

### 5.1 Keyboard Controller (`src/swarm_keyboard_controller.py`)

**What it does:** Reads single keystrokes and publishes command strings to a ROS topic.

**How it works:**

The terminal is put into "raw mode" so keys are captured instantly without pressing Enter:

```python
def getKey(settings):
    tty.setraw(sys.stdin.fileno())                    # Raw mode — instant key capture
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Wait 0.1s for a keypress
    if rlist:
        key = sys.stdin.read(1)                       # Read exactly 1 character
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore terminal
    return key
```

Two dictionaries map keys to command strings:

```python
moveBindings = { 'w': 'FORWARD', 's': 'BACKWARD', ... }
commandBindings = { 'v': 'V', 't': 'TAKEOFF', ... }
```

The `KeyboardController` node publishes to `/swarm/command`:

```python
self.publisher_ = self.create_publisher(String, '/swarm/command', 10)
```

**How to modify:**

- **Add a new key binding:** Add an entry to `moveBindings` or `commandBindings`. Example — bind `g` to a new "GRID" formation:
  ```python
  commandBindings = {
      ...
      'g': 'GRID',    # ← add this
  }
  ```
  Then handle `"GRID"` in the Orchestrator's `command_callback` (see 5.2).

- **Change movement step size:** The step size (1 meter) is defined in the Orchestrator, not here. This file only sends the command string.

---

### 5.2 Swarm Orchestrator (`src/swarm_orchestrator.py`)

**What it does:** The brain. Receives commands, maintains the swarm's virtual center point, calculates where each drone should go, and dispatches GOTO commands.

**Key state variables:**

```python
self.center_x = 10.0          # Swarm center — North (meters)
self.center_y = 0.0           # Swarm center — East (meters)
self.center_z = -8.0          # Swarm center — altitude (negative = up in NED)
self.center_yaw = 0.0         # Heading (degrees)
self.current_formation = None  # "V" | "LINE" | None
self.active_swarm = set(...)  # Drones currently in formation
self.detached_drones = set()  # Drones controlled independently
```

**Independent Control (Detachment):**
If you press `1` through `9`, the orchestrator moves that drone into the `detached_drones` list. Manual commands (`w`, `a`, `s`, `d`) will then apply **only** to that drone. Meanwhile, the background formation loops (like orbit or V) will continue running but will purposely skip sending commands to detached drones, leaving a gap in the formation. Pressing `p` moves the drone back into the `active_swarm`, causing it to immediately rejoin.

**How commands flow — `command_callback()`:**

Every message on `/swarm/command` arrives here. It's a big if/elif chain:

```python
def command_callback(self, msg):
    cmd = msg.data.upper()

    if cmd == "FORWARD":
        self.center_x += 1.0          # Move center 1m North
        self.update_formation()        # Recalculate all drone positions

    elif cmd == "V":
        self.current_formation = "V"
        self.update_formation()
    ...
```

**How to modify:**

- **Change the movement step size:** Change `1.0` to any value. For example, `self.center_x += 5.0` makes each `w` press move 5 meters.

- **Change default altitude:** Modify `self.center_z = -8.0` in `__init__`. Remember: NED coordinate system, so `-10.0` = 10 meters high, `-20.0` = 20 meters high.

**Spawn offset compensation — `send_command()`:**

Each drone spawns at a different X position (5m apart). When sending GOTO, the orchestrator subtracts the drone's spawn offset so all math uses a shared global frame:

```python
def send_command(self, drone_id, command_str, x=0.0, y=0.0, z=0.0, yaw=0.0, quiet=False):
    if command_str == "GOTO":
        offset_x, offset_y = self.spawn_offsets.get(drone_id, (0.0, 0.0))
        req.x = float(x - offset_x)    # Global → drone-local
        req.y = float(y - offset_y)
```

**How to modify:** If you change `SPAWN_SPACING_X` in `start_swarm.sh`, also update the `SPAWN_SPACING_X` env var (line 19) or the `PX4_GZ_MODEL_POSE` values in the shell script to match.

**Formation math — `form_v()`:**

```python
def form_v(self, center_x, center_y, center_z, spacing=5.0):
    # Leader at center
    self.send_command(1, "GOTO", center_x, center_y, center_z)

    wing_pair = 1
    for i in range(2, self.num_drones + 1, 2):
        # Left wing (even drones): behind and left
        self.send_command(i, "GOTO",
            center_x - spacing * wing_pair,
            center_y - spacing * wing_pair, center_z)
        # Right wing (odd drones): behind and right
        if i + 1 <= self.num_drones:
            self.send_command(i + 1, "GOTO",
                center_x - spacing * wing_pair,
                center_y + spacing * wing_pair, center_z)
        wing_pair += 1
```

**How to modify — add a new formation:**

1. Write a new method (e.g., `form_diamond`):
   ```python
   def form_diamond(self, cx, cy, cz, spacing=5.0):
       self.send_command(1, "GOTO", cx + spacing, cy, cz)        # Front
       self.send_command(2, "GOTO", cx, cy - spacing, cz)        # Left
       self.send_command(3, "GOTO", cx, cy + spacing, cz)        # Right
       # Add more drones as needed
   ```

2. Handle it in `update_formation()`:
   ```python
   def update_formation(self):
       if self.current_formation == "V":
           self.form_v(...)
       elif self.current_formation == "DIAMOND":    # ← add this
           self.form_diamond(self.center_x, self.center_y, self.center_z)
   ```

3. Handle the command in `command_callback()`:
   ```python
   elif cmd == "DIAMOND":
       self.current_formation = "DIAMOND"
       self.update_formation()
   ```

4. Add a key binding in the keyboard controller (see 5.1).

**Revolution system — `_revolution_tick()`:**

Runs at 10 Hz via a ROS timer. Moves all drones in a circle:

```python
def _revolution_tick(self):
    self.revolution_angle += self.revolution_speed * self._tick_dt  # Advance angle

    for i in range(1, self.num_drones + 1):
        drone_angle_rad = math.radians(self.revolution_angle + offset)
        x = self.center_x + self.orbit_radius * math.cos(drone_angle_rad)
        y = self.center_y + self.orbit_radius * math.sin(drone_angle_rad)
        self.send_command(i, "GOTO", x, y, self.center_z, yaw, quiet=True)
```

**How to modify:**
- **Change orbit radius:** `self.orbit_radius = 15.0` in `__init__`
- **Change orbit speed:** `self.revolution_speed = 10.0` (degrees/second)
- **Change tick rate:** `self._tick_dt = 0.1` (0.1s = 10 Hz)

---

### 5.3 Swarm Controller (`src/swarm_controller.py`)

**What it does:** Bridges ROS 2 to MAVSDK. Creates per-drone ROS services, receives commands from the Orchestrator, and dispatches them to the SwarmManager.

**Key architecture detail:** MAVSDK is async (Python `asyncio`), but ROS 2 service callbacks are synchronous. The controller solves this by running an asyncio event loop on a background thread:

```python
self.loop = asyncio.new_event_loop()
threading.Thread(target=self.loop.run_forever, daemon=True).start()

# Connect all drones on that loop
asyncio.run_coroutine_threadsafe(self.swarm.connect_all_drones(), self.loop)
```

**Per-drone service creation:**

```python
for i in range(1, NUM_DRONES + 1):
    self.create_service(Command, f"/drone_{i}/mission_command",
        lambda req, res, drone_id=i: self.handle_command(req, res, drone_id))
```

**Command handler:**

```python
def handle_command(self, request, response, drone_id):
    asyncio.run_coroutine_threadsafe(
        self.swarm.execute_command(drone_id, request.command, request.x, ...),
        self.loop)
    response.response = f"Command {request.command} accepted"
    return response
```

**Telemetry callback — forwards drone position to ROS topic + Prometheus:**

```python
def telemetry_cb(self, drone_id, x, y, z):
    msg = Point(x=float(x), y=float(y), z=float(z))
    self.telemetry_pubs[drone_id].publish(msg)
    if self.prometheus_enabled:
        self.metrics_altitude.labels(drone_id=str(drone_id)).set(z)
```

**How to modify:**
- **Add a new metric:** Create a new Prometheus `Gauge` or `Counter` in `__init__`, then update it in the relevant callback.
- **Change the number of drones:** Set the `NUM_DRONES` environment variable before running. The controller reads it from `os.environ`.

---

### 5.4 SwarmManager Package (`src/manager/`)

This is the core drone control logic, split into sub-modules:

#### `manager.py` — Entry Point

Composes all sub-modules. Maintains a `drones` dict:

```python
drones[drone_id] = {
    "system": System(port=grpc_port),     # MAVSDK connection
    "port": udp_port,                      # 14540, 14541, ...
    "target_position": PositionNedYaw | None,  # Current flight target
    "setpoint_task": asyncio.Task | None,  # Background streaming task
    "is_global_position_ok": bool,         # GPS lock status
    "is_home_position_ok": bool,           # Home position established
    "home_alt": float,                     # Absolute altitude at home
}
```

`execute_command()` validates against `VALID_COMMANDS` then calls the right function:

```python
VALID_COMMANDS = {"ARM", "TAKEOFF", "LAND", "GOTO", "START_OFFBOARD", "HOVER"}
```

**How to modify — add a new command:**

1. Add it to `VALID_COMMANDS` in `commands.py`
2. Write the handler function in `commands.py`
3. Add the dispatch in `manager.py`'s `execute_command()`:
   ```python
   elif command == "MY_NEW_COMMAND":
       await my_new_command(self.drones, drone_id, self._logger)
   ```

#### `connection.py` — Drone Connection

`connect_all_drones()` connects to all drones **concurrently** via `asyncio.gather()`. Per drone:

```python
await drone.connect(system_address=f"udpin://0.0.0.0:{port}")

async for state in drone.core.connection_state():
    if state.is_connected:
        break
```

After connection, fires callbacks to start telemetry/battery/health/home monitor loops.

#### `commands.py` — Flight Commands

**Takeoff sequence (`takeoff_drone`)** — the most complex function:

```python
# 1. Wait for GPS lock (poll every 2s, 120s timeout)
while not (drones[id]["is_global_position_ok"] and drones[id]["is_home_position_ok"]):
    await asyncio.sleep(2)

# 2. Clear stale flight state
await drone.action.hold()

# 3. Set relative takeoff altitude
await drone.action.set_takeoff_altitude(8.0)    # 8m above ground

# 4. Arm motors (3 retries)
await drone.action.arm()

# 5. Take off
await drone.action.takeoff()
```

**How to modify:**
- **Change takeoff altitude:** Change `8.0` in `takeoff_drone()`
- **Change health timeout:** Change `health_wait_timeout = 120` (seconds)
- **Change arm retries:** Change the `range(3)` loop count

**Position validation (`validate_position`):**

```python
MAX_POSITION_M = 500.0     # Max X/Y (meters)
MAX_ALTITUDE_M = 100.0     # Max altitude (min NED Z)

def validate_position(x, y, z):
    x = clamp(float(x), -500, 500)
    y = clamp(float(y), -500, 500)
    z = clamp(float(z), -100, 10)    # -100m up, +10m underground
    return x, y, z
```

**How to modify:** Change `MAX_POSITION_M` or `MAX_ALTITUDE_M` at the top of `commands.py`.

#### `setpoint.py` — Offboard Heartbeat

PX4 **requires** a command every ~500ms to stay in OFFBOARD mode. This loop runs at 10 Hz:

```python
async def setpoint_loop(drones, drone_id, logger):
    while True:
        target = drones[drone_id].get("target_position")
        if target is not None:
            await drone.offboard.set_position_ned(target)     # Fly to target
        else:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))  # Hover
        await asyncio.sleep(0.1)    # 10 Hz
```

**How to modify:**
- **Change update rate:** Change `0.1` (but keep ≤ 0.5s or PX4 exits offboard)
- **Add velocity-based control:** Replace `set_position_ned` with `set_velocity_ned` for velocity-mode flight

#### `telemetry.py` — Background Monitors

Four async loops per drone:

| Loop | Streams | Action |
|---|---|---|
| `telemetry_loop` | `position_velocity_ned()` | Forwards (N, E, D) to callbacks |
| `battery_monitor_loop` | `battery()` | RTL if `remaining_percent < 0.20` |
| `health_monitor_loop` | `health()` | Caches GPS/home status in `drones` dict |
| `home_monitor_loop` | `home()` | Caches `absolute_altitude_m` |

**How to modify:**
- **Change battery threshold:** Change `BATTERY_CRITICAL_THRESHOLD = 0.20` at the top of the file
- **Add a new telemetry stream:** Write a new async loop following the same pattern, register it as a callback in `manager.py`'s `connect_all_drones()`

---

### 5.5 Telemetry Logger (`src/swarm_telemetry_logger.py`)

Subscribes to `/drone_N/telemetry` for all drones, writes CSV:

```python
self.csv_writer.writerow(["Timestamp", "DroneID", "X", "Y", "Z"])

def log_telemetry(self, drone_id, msg):
    self.csv_writer.writerow([time.time(), drone_id, msg.x, msg.y, msg.z])
    self.csv_file.flush()    # Crash-safe: data is written immediately
```

Files are saved to `flight_logs/swarm_flight_log_{unix_timestamp}.csv`.

**How to modify:**
- **Add columns:** Add fields to the header row and the `writerow()` call. For example, add yaw by subscribing to a yaw topic.
- **Change log directory:** Change `log_dir = "flight_logs"` to any path.

---

### 5.6 Hardware Abstraction Layer (`src/hal/`)

**`abstract_hal.py`** defines 13 methods every drone backend must implement (connect, arm, takeoff, land, set_position, stream telemetry, etc.).

**`sitl_hal.py`** implements them using MAVSDK for Gazebo simulation. This is what runs today.

**`pixhawk_hal.py`** is a stub for real hardware — all methods raise `NotImplementedError`.

**How to modify — add support for new hardware:**

1. Create a new file (e.g., `my_custom_hal.py`)
2. Subclass `AbstractDroneHAL`
3. Implement all 13 methods
4. Swap the import in `hal/__init__.py`

---

## 6. Changing the Number of Drones

This requires changes in **two places**:

**1. Environment variable:**
```bash
export NUM_DRONES=5    # Before running, or edit start_swarm.sh
```

**2. Shell script** (`start_swarm.sh`) — add PX4 launch commands for drones 4 and 5:
```bash
# Drone 4
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="15,0,0,0,0,0" \
  build/px4_sitl_default/bin/px4 -i 3 build/px4_sitl_default/etc &
sleep 10

# Drone 5
PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="20,0,0,0,0,0" \
  build/px4_sitl_default/bin/px4 -i 4 build/px4_sitl_default/etc &
```

Everything else (formations, services, telemetry) auto-scales from `NUM_DRONES`.

---

## 7. Configuration Reference

| Variable | Default | Where | What to change |
|---|---|---|---|
| `NUM_DRONES` | 3 | Env var / `start_swarm.sh` | Number of drones |
| `SPAWN_SPACING_X` | 5.0 | Env var / orchestrator | Gap between drones at spawn |
| `center_z` | -8.0 | `swarm_orchestrator.py:49` | Default flight altitude |
| `orbit_radius` | 15.0 | `swarm_orchestrator.py:58` | Revolution circle size |
| `revolution_speed` | 10.0 | `swarm_orchestrator.py:56` | Orbit speed (°/s) |
| `rotation_speed` | 30.0 | `swarm_orchestrator.py:63` | Yaw spin speed (°/s) |
| `BATTERY_CRITICAL_THRESHOLD` | 0.20 | `manager/telemetry.py:7` | RTL battery level (0–1) |
| `MAX_POSITION_M` | 500.0 | `manager/commands.py:13` | Max flight distance |
| `MAX_ALTITUDE_M` | 100.0 | `manager/commands.py:14` | Max altitude |
| Takeoff altitude | 8.0 | `manager/commands.py:97` | Height after takeoff |

---

## 8. Running Tests

```bash
cd ~/remaking_skyflock
python -m pytest src/my_digirc/tests/ -v
```

Tests run **without ROS or Gazebo** — they test pure math and validation logic:

- **`test_formation_math.py`** — V and Line position calculations, symmetry, scaling, offset subtraction
- **`test_command_validation.py`** — Valid command set, clamping, boundary values, type coercion
