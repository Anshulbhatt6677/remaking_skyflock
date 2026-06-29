# 🛩️ DigiRC Developer Handbook — Complete Beginner-to-Production Guide

> **Read this if you know basic Python but almost nothing about ROS, Gazebo, PX4,
> MAVLink, or drone control.** This is your onboarding handbook, robotics course,
> engineering wiki, implementation guide, debugging manual, and rebuild playbook —
> all in one file. It teaches the concepts *and* the actual code in
> `v2_swarm/src/digi_rc/`, line by line where it matters.
>
> **Everything here is based on the real implementation**
> ([`src/multi_drone.py`](src/multi_drone.py), [`srv/Command.srv`](srv/Command.srv),
> [`CMakeLists.txt`](CMakeLists.txt), [`package.xml`](package.xml)) — not on
> assumptions.

---

### ⚠️ Two corrections before you start (read these first)

The robotics world has a lot of similarly-named tools. Two myths to kill *now*,
because getting them wrong will confuse everything else:

> **MYTH 1: "DigiRC uses MAVROS."**
> ❌ **It does not.** DigiRC talks to the autopilot through **MAVSDK** (a modern
> Python SDK that speaks the MAVLink protocol over gRPC). We will still teach you
> *what* MAVROS is (§2.4) because you'll hear about it constantly — but DigiRC
> deliberately chose MAVSDK instead. The difference, and why, is in §2.4 and §12.

> **MYTH 2: "This is ROS 1 (rospy, catkin, rosrun, roscore)."**
> ❌ **It's ROS 2 (Humble).** We use `rclpy` (not `rospy`), `colcon` (not
> `catkin`), `ros2 run` (not `rosrun`), and there is **no ROS master / roscore** —
> ROS 2 uses a peer-to-peer system called **DDS**. We'll map the old terms to the
> new ones as we go.

Proof, straight from the package's own metadata:

```xml
<!-- package.xml -->
<description>MAVSDK Offboard Control Package</description>
<depend>rclpy</depend>          <!-- ROS 2 Python, not rospy -->
<buildtool_depend>ament_cmake</buildtool_depend>   <!-- ROS 2 build, not catkin -->
```

```python
# multi_drone.py — line 8
from mavsdk import System          # ← MAVSDK, not mavros
```

Good. Now let's build your understanding from zero.

---

## 📑 Table of Contents

1. [Introduction](#1-introduction)
2. [Beginner Robotics Fundamentals](#2-beginner-robotics-fundamentals)
3. [Python Fundamentals Used in DigiRC](#3-python-fundamentals-used-in-digirc)
4. [Full Repository Walkthrough](#4-full-repository-walkthrough)
5. [End-to-End Data Flow](#5-end-to-end-data-flow)
6. [Step-by-Step Rebuild Guide](#6-step-by-step-rebuild-guide)
7. [Environment Setup](#7-environment-setup)
8. [Creating Your First ROS 2 Node](#8-creating-your-first-ros-2-node)
9. [Building a Basic Drone Controller](#9-building-a-basic-drone-controller)
10. [Building DigiRC Core](#10-building-digirc-core)
11. [ROS Topics & Services Deep Dive](#11-ros-topics--services-deep-dive)
12. [MAVSDK Deep Dive (and MAVROS comparison)](#12-mavsdk-deep-dive)
13. [Gazebo Testing Workflow](#13-gazebo-testing-workflow)
14. [Live Arm & Flight Testing SOP](#14-live-arm--flight-testing-sop)
15. [Debugging Handbook](#15-debugging-handbook)
16. [Codebase Engineering Standards](#16-codebase-engineering-standards)
17. [Swarm Architecture Relation](#17-swarm-architecture-relation)
18. [Advanced Improvements](#18-advanced-improvements)
19. [Developer Exercises](#19-developer-exercises)
20. [Final Production Blueprint](#20-final-production-blueprint)

---

# 1. Introduction

## 1.1 What is DigiRC?

**DigiRC** ("Digital Remote Control") is a **ROS 2 package** that turns
high-level commands like *"arm drone 1"* or *"fly drone 3 to (10, 5, 8)"* into the
low-level autopilot calls that actually move motors. It is the **single lowest
layer of the swarm's *software*** — the only part that speaks directly to the
flight controller.

Think of a traditional drone setup: a human holds a physical radio transmitter
(an "RC" — remote control) with sticks. DigiRC is the **digital, software version
of that transmitter**: instead of a human moving sticks, *other programs* (or a
web UI, or you typing in a terminal) send commands over the network, and DigiRC
translates them into flight commands. Hence "Digital RC."

It is implemented in a **single Python file**, [`src/multi_drone.py`](src/multi_drone.py),
containing one main class: **`SwarmController`**.

## 1.2 Why does it exist? What problem does it solve?

Controlling even one drone correctly is hard:
- You must speak the drone's protocol (MAVLink).
- You must manage connection state, GPS lock, arming safety, and flight modes.
- You must stream position commands continuously (PX4 "OFFBOARD" mode demands it).
- You must convert between coordinate frames (drones think in "NED"; humans think
  "up is positive").

Now multiply that by **5+ drones running at once**, each needing its own
connection, its own command channel, and its own telemetry stream — *concurrently*.

DigiRC solves exactly this: **it gives every drone a clean, identical
network interface** (one ROS 2 *service* to command it, ROS 2 *topics* to read
its telemetry) and hides all the MAVSDK/MAVLink/asyncio complexity behind it.

**The core promise:** *the same commands work in simulation and on real hardware.*
You develop and test in Gazebo, then flip one environment variable
(`DRONE_MODE=real`) and the exact same code flies a real Pixhawk.

## 1.3 How it fits into the swarm system

```
┌──────────────────────────────────────────────────────────────────────┐
│  flyMe  (web UI + FastAPI backend)        ← humans click buttons here  │
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  ROS 2 service calls + topic reads
┌───────────────────────────────▼──────────────────────────────────────┐
│  swarm_orchestrator  (formations, navigation, ML, autonomy)           │
│   BehaviorManager → MissionExecutor → (ROS 2 service calls)            │
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  /drone_N/mission_command  (Command.srv)
┌───────────────────────────────▼──────────────────────────────────────┐
│  ★ DigiRC  (src/digi_rc/multi_drone.py)  ← YOU ARE HERE                │
│    SwarmController node:                                               │
│      • Service per drone:  /drone_N/mission_command                    │
│      • Publishes telemetry: /drone_N/position, /armed, /velocity ...   │
│      • Bridges ROS 2  ⇄  asyncio  ⇄  MAVSDK                            │
└───────────────────────────────┬──────────────────────────────────────┘
                                 │  MAVSDK (gRPC) → MAVLink
┌───────────────────────────────▼──────────────────────────────────────┐
│  PX4 autopilot   (SITL in Gazebo  OR  real Pixhawk board)             │
│    runs flight modes, stabilizes, drives the motors                    │
└────────────────────────────────────────────────────────────────────────┘
```

**The key boundary:** everything *above* DigiRC is platform-agnostic and never
knows whether a drone is real or simulated. DigiRC is the translator that makes
that possible.

## 1.4 Relationship between the pieces (one-liners)

| Piece | What it is | DigiRC's relationship to it |
|---|---|---|
| **ROS 2** | A messaging framework so programs can talk. | DigiRC *is* a ROS 2 node; it exposes services/topics. |
| **MAVLink** | The wire protocol drones speak. | DigiRC speaks it *indirectly*, via MAVSDK. |
| **MAVSDK** | A library that speaks MAVLink with friendly `async` Python calls. | DigiRC's actual control library (`from mavsdk import System`). |
| **MAVROS** | An *alternative* ROS↔MAVLink bridge. | **Not used.** DigiRC uses MAVSDK instead (§2.4). |
| **PX4** | The autopilot firmware (the drone's "brain"). | DigiRC commands PX4 through MAVSDK. |
| **Gazebo** | A 3D physics simulator. | Hosts simulated drones+PX4 for testing. |
| **SITL** | "Software In The Loop" = PX4 running on your PC. | What DigiRC connects to in simulation. |

## 1.5 High-level architecture of DigiRC itself

```
                      ┌─────────────────────────────────────────────┐
                      │       SwarmController  (one ROS 2 node)      │
                      │            node name: swarm_controller_node  │
                      ├─────────────────────────────────────────────┤
   ros2 service call  │  ┌───────────────────────────────────────┐  │
   ──────────────────►│  │ Service callbacks (ROS thread)         │  │
   /drone_N/          │  │   handle_command()  → returns fast      │  │
   mission_command    │  └───────────────┬───────────────────────┘  │
                      │                   │ run_coroutine_threadsafe │
                      │  ┌────────────────▼──────────────────────┐  │
                      │  │ asyncio event loop (separate thread)   │  │
                      │  │   execute_drone_command()              │  │
                      │  │     arm/takeoff/goto/land/rtl ...      │  │
                      │  │   telemetry_publisher_task() ×N @10Hz  │  │
                      │  └────────────────┬──────────────────────┘  │
                      │                   │ MAVSDK async calls       │
   ◄──────────────────│  ┌────────────────▼──────────────────────┐  │
   /drone_N/position  │  │ Publishers (telemetry out)             │  │
   /drone_N/armed     │  │   armed, position, velocity,           │  │
   /drone_N/velocity  │  │   telemetry_frame, swarm_status        │  │
   ...                │  └───────────────────────────────────────┘  │
                      └─────────────────────────┬───────────────────┘
                                                │ MAVSDK (gRPC :50041+)
                            ┌───────────────────▼───────────────────┐
                            │ mavsdk_server (sim)  OR  serial (real) │
                            └───────────────────┬───────────────────┘
                                                │ MAVLink (UDP :14541+ / serial)
                                       ┌────────▼────────┐
                                       │   PX4 autopilot  │
                                       └─────────────────┘
```

Hold this diagram in your head. The rest of the handbook zooms into each box.

## 1.6 Full data flow (preview — details in §5)

**Command path (down):**
```
You / flyMe / orchestrator
   → ros2 service call /drone_3/mission_command {command: "GOTO", x:10, y:5, z:8}
   → handle_command()        [validates, returns "executing" immediately]
   → execute_drone_command() [async, on the event loop]
   → goto_drone()            [convert frame, send setpoint]
   → drone.offboard.set_position_ned(...)   [MAVSDK]
   → MAVLink message → PX4 → motors → drone moves
```

**Telemetry path (up):**
```
PX4 estimates position
   → drone.telemetry.position_velocity_ned()   [MAVSDK stream]
   → _get_position()         [convert NED→global, flip altitude sign]
   → publisher.publish(Point) on /drone_3/position   [10 Hz]
   → orchestrator / flyMe / your `ros2 topic echo` reads it
```

---

# 2. Beginner Robotics Fundamentals

This section assumes **zero** robotics knowledge. We build every concept from the
ground up and immediately show how DigiRC uses it.

## 2.1 What is ROS (and ROS 2)?

**ROS** = *Robot Operating System*. The name is misleading: **it is not an
operating system.** It is a **framework + set of conventions for letting many
small programs talk to each other** on a robot. ROS 2 is the modern rewrite
(this project uses **ROS 2 Humble**).

**Why does it exist?** A robot is many subsystems — perception, planning, control,
sensors — written by different people, often in different languages. ROS gives
them a *common language* and *plumbing* to exchange data without hard-wiring them
together.

### Core concepts (each explained + DigiRC example)

#### Node
**What:** a single running program that participates in ROS. **Why:** modularity —
each node does one job and can be started/stopped independently.
**DigiRC:** the whole `SwarmController` is one node named `swarm_controller_node`:

```python
# multi_drone.py
class SwarmController(Node):              # inherits from rclpy Node
    def __init__(self):
        super().__init__("swarm_controller_node")   # registers the node name
```

> **ROS 1 → ROS 2 map:** there is **no `roscore`/ROS master** in ROS 2. Nodes find
> each other automatically via DDS (a peer-to-peer discovery protocol). In ROS 1
> you had to start `roscore` first; in ROS 2 you never do.

#### Topic
**What:** a named channel carrying a *continuous stream* of messages. Many
publishers, many subscribers, **fire-and-forget** (no reply). **Why:** perfect for
sensor data and telemetry that flows constantly.
**DigiRC:** `/drone_1/position` carries the drone's location 10 times per second.

#### Publisher
**What:** the "sender" side of a topic. **Why:** a node announces "I will produce
this kind of data on this channel."
**DigiRC:**

```python
# create a publisher: I will send Point messages on /drone_1/position, queue=10
self.position_publishers[drone_id] = self.create_publisher(Point, f"drone_{drone_id}/position", 10)
# ...later, actually send one:
self.position_publishers[drone_id].publish(pos)
```

#### Subscriber
**What:** the "receiver" side of a topic. You register a **callback** function that
ROS calls every time a message arrives. **Why:** react to incoming data.
**DigiRC:** it subscribes to speed commands and the orchestrator heartbeat:

```python
self.speed_subscribers[drone_id] = self.create_subscription(
    Float32,                                  # message type
    f"drone_{drone_id}/speed_command",        # topic name
    lambda msg, did=drone_id: self.speed_callback(msg, did),  # callback
    10                                        # queue depth
)
```

> 🧠 **Why the `lambda ... did=drone_id`?** A subtle but critical Python detail.
> We're creating one subscriber per drone in a loop. The `did=drone_id` "captures"
> the current loop value so each callback remembers *its own* drone id. Without it,
> all callbacks would share the last loop value — a classic bug. (More in §3.)

#### Service
**What:** a **request → response** call. One client asks, one server answers, the
client **waits** for the reply. **Why:** for actions where you need confirmation,
not a stream.
**DigiRC:** `/drone_N/mission_command` is a service. You send `{command:"ARM"}` and
get back `"executing"`:

```python
self.drone_services[drone_id] = self.create_service(
    Command,                              # service type (from srv/Command.srv)
    f"drone_{drone_id}/mission_command",  # service name
    lambda req, resp, did=drone_id: self.handle_command(req, resp, did)
)
```

> **Topic vs Service — the rule of thumb:**
> - **Topic** = "weather broadcast." Continuous, no reply. (telemetry)
> - **Service** = "ordering at a counter." One request, one reply, you wait.
>   (commands)

#### Parameter
**What:** a named, typed setting on a node that can be **read and changed at
runtime** from the terminal. **Why:** tune behavior without editing/restarting code.
**DigiRC** itself reads parameters mostly via environment variables, but the
*orchestrator* layer above it uses ROS parameters heavily (e.g. live formation
radius). Example of the mechanism:

```bash
ros2 param set /behavior_manager radius 15.0   # change a live setting
```

#### Message
**What:** the typed *shape* of data on a topic/service. **Why:** so sender and
receiver agree on the format.
**DigiRC uses these standard messages:**

| Message | Fields | Used by |
|---|---|---|
| `std_msgs/Bool` | `data: bool` | `/drone_N/armed` |
| `geometry_msgs/Point` | `x, y, z` (float64) | `/drone_N/position` |
| `geometry_msgs/Vector3` | `x, y, z` (float64) | `/drone_N/velocity` |
| `std_msgs/String` | `data: string` | `/swarm_status`, `/drone_N/telemetry_frame` |
| `std_msgs/Float32` | `data: float32` | `/drone_N/speed_command` |

Plus one **custom** service message it defines itself, `Command.srv` (§4.4).

#### Launch files
**What:** files that start *many* nodes at once with preset configuration. **Why:**
real systems have dozens of nodes; you don't start them by hand.
**In this project:** the `.sh` launch scripts in `v2_swarm/` (e.g.
`launch_gazebo_for_flyme.sh`) play this role — they start Gazebo, PX4, DigiRC, and
helper nodes together. (ROS 2 also has Python/XML launch files; this project leans
on shell scripts.)

#### Workspace
**What:** a folder where ROS 2 packages live and get built together. **Why:**
organizes and compiles your code. In ROS 2 you **build with `colcon`** (ROS 1 used
`catkin`):

```bash
cd ~/project/v2_swarm
colcon build --symlink-install        # compile all packages
source install/setup.bash             # make them usable in this terminal
```

#### The ROS graph
**What:** the live network of nodes connected by topics/services. **Why:** it's
your mental model + a debugging tool. Inspect it:

```bash
ros2 node list        # all running nodes
ros2 topic list       # all channels
rqt_graph             # GUI: see nodes and connections visually
```

## 2.2 What is MAVLink?

**MAVLink** (Micro Air Vehicle Link) is the **lightweight messaging protocol that
drones speak.** When your flight controller sends "battery is at 80%" or you send
"arm the motors," that's a MAVLink message traveling over a radio/USB/UDP link.

**Why it exists:** drones have low-bandwidth radio links. MAVLink packs commands
and telemetry into tiny, efficient binary packets (as small as 8–263 bytes).

Key message categories you'll meet:
- **Heartbeat** — "I'm alive," sent ~1 Hz. Loss of heartbeat = lost link.
- **Telemetry** — position, attitude, battery, GPS, velocity.
- **Commands** — arm, takeoff, set mode, go to position.
- **RC override / setpoints** — continuous control inputs.

**The communication pipeline:**
```
PX4 (speaks MAVLink) ⇄ link (UDP/serial) ⇄ something that also speaks MAVLink
```
That "something" can be QGroundControl (a GUI), MAVROS, or — in our case —
**MAVSDK**.

> You almost never write raw MAVLink by hand. You use a library (MAVSDK) that
> builds and parses these messages for you.

## 2.3 What is PX4?

**PX4** is **open-source autopilot firmware** — the software that runs *on the
drone's flight-controller board* (e.g. a Pixhawk) and actually keeps the drone in
the air. It is the drone's low-level brain.

PX4 handles:
- **Stabilization** — reading the IMU (gyro/accelerometer) hundreds of times per
  second and adjusting motors to stay level.
- **Flight modes** — different control behaviors (see below).
- **Arming logic** — safety checks before motors can spin.
- **Mission execution, failsafes, sensor fusion (EKF), GPS, etc.**

### Flight modes (the ones DigiRC cares about)

| PX4 mode | What it means | DigiRC relevance |
|---|---|---|
| **OFFBOARD** | PX4 obeys position/velocity setpoints from an *external computer*. | **DigiRC lives here.** All our movement uses OFFBOARD. |
| **POSCTL** | Position hold from RC sticks. | Drone may fall back here if OFFBOARD drops. |
| **HOLD/LOITER** | Hover in place. | — |
| **RTL** | Return To Launch (auto-fly home & land). | DigiRC's `RTL` command triggers this. |
| **LAND** | Descend & disarm. | DigiRC's `LAND` command. |

> 🔑 **The single most important PX4 rule for DigiRC:** **OFFBOARD mode requires a
> continuous stream of setpoints (≥2 Hz).** If the stream stops, PX4 leaves OFFBOARD
> for safety. This *one rule* explains why DigiRC sends setpoints in loops (e.g.
> "send 5 setpoints before starting offboard," and why the orchestrator above
> re-sends targets at 5 Hz). Remember it.

### SITL — Software In The Loop

**SITL** means running **the exact same PX4 firmware as a program on your PC**
instead of on a physical board. The motors/sensors are simulated (by Gazebo), but
the autopilot logic is *identical* to the real thing. This is how you test safely.

## 2.4 What is MAVROS — and why DigiRC does NOT use it

**MAVROS** is a ROS package that acts as a **bridge between ROS and MAVLink**. It
connects to a PX4 drone, then *re-publishes* all MAVLink telemetry as ROS topics
(like `/mavros/state`, `/mavros/local_position/pose`) and lets you command the
drone by publishing to ROS topics / calling ROS services (like
`/mavros/setpoint_position/local`, `/mavros/cmd/arming`).

```
   ┌──── The MAVROS approach (NOT used here) ────┐
   ROS topics  ⇄  MAVROS node  ⇄  MAVLink  ⇄  PX4
   /mavros/...     (the bridge)
```

**DigiRC instead uses MAVSDK directly:**

```
   ┌──── The MAVSDK approach (what DigiRC does) ────┐
   our Python  ⇄  MAVSDK (System)  ⇄  MAVLink  ⇄  PX4
   async calls    drone.action.arm()
```

| | MAVROS | MAVSDK (DigiRC's choice) |
|---|---|---|
| Style | ROS topics/services for everything | Python `async`/`await` API |
| Per-drone setup | Namespaced topics, one MAVROS node per drone | One `System()` object per drone |
| Multi-drone | Heavier; many topics | Lightweight; explicit objects |
| Where it runs | A separate ROS node | Inside our own node, on an asyncio loop |
| API feel | "publish to a setpoint topic" | "`await drone.offboard.set_position_ned(...)`" |

**Why DigiRC chose MAVSDK:** explicit per-drone objects make multi-drone control
clean (`self.drone_instances[drone_id]`), the `async` API maps naturally to "do
many drones at once," and it avoids running a separate MAVROS bridge per drone.

> ✅ **Takeaway:** Whenever a tutorial mentions `/mavros/...` topics, mentally
> translate to "DigiRC does this with a MAVSDK call instead." You will *not* find
> MAVROS topics in this codebase.

## 2.5 What is Gazebo?

**Gazebo** is a **3D physics simulator** for robots. It simulates gravity,
collisions, sensors (cameras, GPS, IMU), and the drone's physical body, so you can
test flight software without risking real hardware.

Concepts:
- **World** — the environment (ground, buildings, obstacles). This project has
  `worlds/nav_obstacles.world` etc.
- **Model** — a robot/object in the world. The drone model here is `iris` (or
  `iris_cam` with cameras).
- **Plugin** — code attached to a model/world to add behavior (e.g. the plugin that
  connects the simulated drone to PX4, or a camera plugin that publishes images).

Gazebo + PX4 SITL together = a complete virtual drone you can fly with the same
DigiRC commands you'd use on real hardware.

---

# 3. Python Fundamentals Used in DigiRC

We teach **only** the Python concepts that actually appear in
[`multi_drone.py`](src/multi_drone.py), each with a real example from the file.

## 3.1 Classes & objects

A **class** is a blueprint; an **object** is one built instance. DigiRC's entire
logic lives in the `SwarmController` class. When the program runs, it creates **one
object** of that class.

```python
class SwarmController(Node):       # blueprint (also inherits Node — see 3.2)
    def __init__(self):            # constructor: runs once when object is created
        super().__init__("swarm_controller_node")
        self.num_drones = 5        # self.xxx = data stored on THIS object
```

- `self` means "this particular object." Every method's first argument is `self`.
- `self.num_drones`, `self.drone_instances`, `self.loop` are **instance
  attributes** — the object's memory.

## 3.2 Inheritance

`class SwarmController(Node)` means SwarmController **inherits** from rclpy's
`Node`. It gets all of `Node`'s abilities (`create_publisher`, `create_service`,
`get_logger`, ...) for free, and adds its own. `super().__init__(...)` calls the
parent's constructor so the ROS machinery initializes properly.

## 3.3 Functions & methods

A **function** is reusable code. A **method** is a function that belongs to a
class. DigiRC methods you'll meet: `handle_command()`, `arm_drone()`,
`goto_drone()`, `connect_all_drones()`. Module-level functions: `main()`,
`spin_ros()`, `run_drone_mission()` (deprecated).

## 3.4 Callbacks

A **callback** is a function you hand to someone else to call *later*, when an
event happens. ROS is built on callbacks: "when a message arrives, call this."

```python
# "When a speed_command arrives, call speed_callback":
self.create_subscription(Float32, topic, self.speed_callback, 10)

def speed_callback(self, msg, drone_id):   # ROS calls this for us
    new_speed = max(0.1, min(5.0, msg.data))   # clamp 0.1–5.0
    self.target_speeds[drone_id] = new_speed
```

You never call `speed_callback` yourself — **ROS calls it for you** when data
arrives. Same for the service handler `handle_command` and the timer callback
`_check_orchestrator_health`.

## 3.5 Loops & comprehensions

DigiRC creates per-drone resources in `for` loops:

```python
for i in range(self.num_drones):
    drone_id = i + 1                      # drones are 1-indexed
    self.armed_publishers[drone_id] = self.create_publisher(...)
```

And uses **list comprehensions** for compact lists:

```python
self.udp_ports  = [14541 + i for i in range(self.num_drones)]   # [14541, 14542, ...]
self.grpc_ports = [50041 + i for i in range(self.num_drones)]
```

## 3.6 Dictionaries

A **dict** maps keys → values. DigiRC stores per-drone state in dicts keyed by
drone id:

```python
self.drone_instances = {}     # {1: <MAVSDK System>, 2: <System>, ...}
self.armed_publishers = {}    # {1: <publisher>, ...}
self.drone_ned_offsets = {}   # {1: (north, east), ...}
```

`self.drone_instances[drone_id]` instantly fetches that drone's MAVSDK object.

## 3.7 Exception handling

`try/except` runs risky code and catches errors instead of crashing. Drone code is
*full* of things that can fail (timeouts, disconnects), so this is everywhere:

```python
try:
    await drone.action.arm()
    await drone.offboard.start()
except (Exception, OffboardError) as error:
    print(f"Failed to arm/start offboard: {error}")
    await drone.action.disarm()       # clean up safely
    return
```

> 🧠 **Why catch errors here?** A single drone failing to arm must **not** crash the
> controller managing 4 other drones. Graceful degradation is a safety property.

## 3.8 Threading

A **thread** is a separate line of execution running "at the same time." DigiRC
uses **two worlds at once**:
1. ROS 2 callbacks run on a **MultiThreadedExecutor** (its own thread).
2. MAVSDK async work runs on an **asyncio event loop** (the main thread).

```python
# main() — multi_drone.py
ros_thread = threading.Thread(target=spin_ros, args=(executor,), daemon=True)
ros_thread.start()           # ROS spins in the background thread

loop = asyncio.new_event_loop()
asyncio.set_event_loop(loop)
node.loop = loop             # asyncio runs on the main thread
```

`daemon=True` means the thread dies automatically when the program exits.

## 3.9 asyncio: `async` / `await` (the heart of DigiRC)

This is the **most important** concept in the file. Talking to a drone involves a
lot of *waiting* (for GPS lock, for arming, for acknowledgements). With 5 drones,
you can't wait for each in turn. `asyncio` lets one thread juggle many waits.

```python
async def arm_drone(self, drone_index):     # "async def" → a coroutine
    ...
    await drone.action.arm()                # "await" → pause here, let others run
    await asyncio.sleep(0.1)                # non-blocking sleep
```

- `async def` defines a **coroutine** (a pausable function).
- `await X` means "pause this coroutine until X is done; meanwhile the event loop
  runs other coroutines." It does **not** freeze the whole program.
- `async for x in stream:` consumes an **async stream** — exactly how MAVSDK
  delivers telemetry:

```python
async for health in drone.telemetry.health():
    if health.is_armable:
        break                # stop reading the stream once we have our answer
```

### The threading ⇄ asyncio bridge (subtle, important)

ROS callbacks (one thread) need to trigger async MAVSDK work (another thread's
event loop). The bridge is **`run_coroutine_threadsafe`**:

```python
def handle_command(self, request, response, drone_id):   # ROS thread, SYNC
    # schedule the slow async work onto the asyncio loop, don't wait for it:
    asyncio.run_coroutine_threadsafe(
        self.execute_drone_command(drone_id, request.command, ...),
        self.loop
    )
    response.response = "executing"     # reply IMMEDIATELY
    return response
```

> 🧠 **Why reply "executing" instead of waiting for the result?** A ROS service
> callback should return fast. Arming takes seconds. So we *schedule* the work and
> return "executing" right away; the actual flight happens asynchronously. The
> caller learns the outcome by watching telemetry (`/drone_N/armed`), not by
> waiting on the service reply.

## 3.10 Timers

A **timer** calls a function on a fixed interval. DigiRC uses one for its safety
watchdog:

```python
self.create_timer(1.0, self._check_orchestrator_health)   # call every 1 second
```

## 3.11 f-strings, environment variables, `lambda`

```python
# f-string: embed values in text
self.get_logger().info(f"Drone {drone_id}: Connecting to UDP {udp_port}")

# environment variable: read config from the shell (with a default)
DRONE_MODE = os.environ.get('DRONE_MODE', 'simulation')

# lambda: a tiny inline function (used to bind drone_id into callbacks)
lambda req, resp, did=drone_id: self.handle_command(req, resp, did)
```

You now know enough Python to read every line of `multi_drone.py`.

---

# 4. Full Repository Walkthrough

The package is small and focused. Here is **every file**, its purpose, and (for the
code files) a deep walkthrough.

```
src/digi_rc/
├── package.xml          # ROS 2 package metadata + dependencies
├── CMakeLists.txt       # build instructions (generates the service, installs node)
├── srv/
│   └── Command.srv      # the custom service message definition
├── src/
│   └── multi_drone.py   # ★ THE ENTIRE IMPLEMENTATION (SwarmController)
├── DIGIRC.md            # older reference doc (partly stale — see notes)
├── API_REFERENCE.md     # service/topic API reference
├── QUICK_START.md       # standalone quick start
└── DIGIRC_DEVELOPER_HANDBOOK.md   # ← this file
```

## 4.1 `package.xml` — what the package *is*

```xml
<name>digi_rc</name>
<description>MAVSDK Offboard Control Package</description>
<buildtool_depend>ament_cmake</buildtool_depend>          <!-- ROS 2 build system -->
<buildtool_depend>rosidl_default_generators</buildtool_depend> <!-- to build Command.srv -->
<depend>rclpy</depend>        <!-- ROS 2 Python client library -->
<depend>std_msgs</depend>     <!-- Bool, String, Float32 -->
<depend>geometry_msgs</depend><!-- Point, Vector3 -->
<member_of_group>rosidl_interface_packages</member_of_group>  <!-- it defines a .srv -->
```

**Role:** tells ROS 2 the package's name, dependencies, and that it generates a
service interface. **If you remove `rclpy`/`std_msgs`/`geometry_msgs`**, the node
won't find its message types and won't build/run.

## 4.2 `CMakeLists.txt` — how it's *built*

```cmake
project(digi_rc)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 1) Turn srv/Command.srv into usable Python/C++ types:
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Command.srv" DEPENDENCIES std_msgs)

# 2) Install the Python node so `ros2 run digi_rc multi_drone.py` can find it:
install(PROGRAMS src/multi_drone.py DESTINATION lib/${PROJECT_NAME})
```

**Role:** two jobs — (1) generate the `Command` service type from the `.srv` file,
(2) install the executable. **If you remove the `rosidl_generate_interfaces` line**,
`from digi_rc.srv import Command` fails — the type wouldn't exist.

> 🧠 **Mixed build note:** this is a `ament_cmake` package (C++-style build) that
> *also* installs Python. That's why there's a `CMakeLists.txt` rather than a pure
> Python `setup.py`. It's done this way because the package **defines a service
> interface** (`.srv`), and interface generation is cleanest under `ament_cmake`.

## 4.3 `srv/Command.srv` — the command contract

```
string command    # request: "ARM" | "DISARM" | "TAKEOFF" | "GOTO" | "LAND" | "RTL" | "OFFBOARD"
float32 x         # request: North metres (NED) — used by GOTO
float32 y         # request: East  metres (NED) — used by GOTO
float32 z         # request: Altitude metres (positive up) — TAKEOFF/GOTO
float32 yaw       # request: Heading radians (0.0 = no change)
---               # ← the separator: everything above = request, below = response
string response   # response: "executing" | "invalid_command" | "drone_not_connected" | "event_loop_error"
```

**Role:** defines the *shape* of every command. The `---` line splits **request**
(what you send) from **response** (what you get back). This is the single most
important interface in the package — it's the contract every caller uses.

> 📝 **Doc-vs-code note:** the older `DIGIRC.md` shows `Command.srv` *without* the
> `yaw` field and lists only 4 commands. The **actual current file** (above) has
> `yaw` and the node accepts **7** commands. Always trust the code.

## 4.4 `src/multi_drone.py` — the implementation (deep walkthrough)

This one file is the entire package logic. Let's walk it top to bottom.

### 4.4.1 Imports (lines 1–18)

```python
import rclpy                                       # ROS 2 Python core
from rclpy.node import Node                        # base class for our node
from rclpy.executors import MultiThreadedExecutor  # runs callbacks on threads
from geometry_msgs.msg import Point, Vector3       # position & velocity messages
from std_msgs.msg import String, Bool, Float32     # status/armed/speed messages
from digi_rc.srv import Command                    # ★ our generated service type
from mavsdk import System                          # ★ MAVSDK — the drone SDK
from mavsdk.offboard import OffboardError, PositionNedYaw  # offboard control types
from mavsdk.telemetry import LandedState           # to detect "on ground"
import asyncio, time, threading, subprocess, signal, os, sys, math
```

Each import maps to a concept you now know: `rclpy`/`Node` = ROS 2; `mavsdk
System` = the per-drone control object; `asyncio`/`threading` = the concurrency
model; `subprocess` = used to launch `mavsdk_server` processes.

### 4.4.2 Module-level config from environment variables (lines 20–30)

```python
DRONE_MODE = os.environ.get('DRONE_MODE', 'simulation')   # 'simulation' or 'real'
REAL_DRONE_CONNECTIONS = os.environ.get('DRONE_CONNECTIONS', '').split(',') if ... else []
MISSION_SPEED = float(os.environ.get('MISSION_SPEED', '1.0'))
```

**Role:** the package is configured by environment variables, not code edits. This
is what makes "sim vs real" a one-line switch. (Full env-var table in §7.6.)

### 4.4.3 `MavsdkDrone` class (lines 32–59) — DEPRECATED

The file explicitly marks this class as **deprecated / kept for reference**.
`SwarmController` manages drones directly now. **Don't use it; don't learn from
it.** Mentioned only so you don't get confused seeing it.

### 4.4.4 `run_drone_mission()` (lines 62–173) — DEPRECATED demo

A standalone scripted mission (takeoff → fly to target → return → land) using
fixed 51/101-step interpolation loops. **It is not used by the service interface.**
It's a legacy demo. (The *old* `DIGIRC.md` describes GOTO as a "101-step
interpolation" — that description came from *this* function, **not** the real
`goto_drone()` used today. See §4.4.10.)

### 4.4.5 `SwarmController.__init__` (lines 176–308) — setup

This builds the whole node. Walk the key parts:

**Mode & drone count:**
```python
self.drone_mode = DRONE_MODE
if self.drone_mode == 'real' and REAL_DRONE_CONNECTIONS:
    self.num_drones = len(REAL_DRONE_CONNECTIONS)   # real: one per connection string
else:
    self.num_drones = int(os.environ.get('NUM_DRONES', '5'))  # sim: default 5
```

**Frame/coordinate state (the global-vs-local trick — see §5.3):**
```python
self.drone_home_gps = {}        # {id: (lat, lon, alt)} each drone's GPS home
self.drone_ned_offsets = {}     # {id: (north, east)} offset from world origin
self.reference_drone_id = 1     # drone 1 is the default reference frame
self._offboard_started = {}     # {id: bool} did we start OFFBOARD yet?
```

**Ports (simulation only):**
```python
self.udp_ports  = [14541 + i for i in range(self.num_drones)]   # MAVLink UDP in
self.grpc_ports = [50041 + i for i in range(self.num_drones)]   # MAVSDK gRPC
```

**Per-drone interfaces (the loop, lines ~246–276):**
```python
for i in range(self.num_drones):
    drone_id = i + 1
    self.drone_services[drone_id]   = self.create_service(Command, f"drone_{drone_id}/mission_command", ...)
    self.armed_publishers[drone_id] = self.create_publisher(Bool,   f"drone_{drone_id}/armed", 10)
    self.position_publishers[drone_id] = self.create_publisher(Point, f"drone_{drone_id}/position", 10)
    self.velocity_publishers[drone_id] = self.create_publisher(Vector3, f"drone_{drone_id}/velocity", 10)
    self.telemetry_frame_publishers[drone_id] = self.create_publisher(String, f"drone_{drone_id}/telemetry_frame", 10)
    self.speed_subscribers[drone_id] = self.create_subscription(Float32, f"drone_{drone_id}/speed_command", ..., 10)
self.status_pub = self.create_publisher(String, "swarm_status", 10)
```

> This loop is the heart of "every drone gets an identical interface." Remove it
> and there is no way to command or monitor drones.

**Safety watchdog (Phase B4, lines ~280–302):**
```python
self._heartbeat_timeout_s = float(os.environ.get("ORCHESTRATOR_HEARTBEAT_TIMEOUT", "3.0"))
if self._heartbeat_active:
    self.create_subscription(String, "/swarm/orchestrator_heartbeat", self._on_orchestrator_heartbeat, 1)
    self.create_timer(1.0, self._check_orchestrator_health)
```

**Start MAVSDK servers (sim only):**
```python
if self.drone_mode == 'simulation':
    self.start_mavsdk_servers()
```

### 4.4.6 `speed_callback` (lines 310–316)

Receives a `Float32` on `/drone_N/speed_command`, **clamps** it to a safe range,
and stores it:

```python
new_speed = max(0.1, min(5.0, msg.data))   # never below 0.1× or above 5×
self.target_speeds[drone_id] = new_speed
```

> The clamp is a safety/validation pattern: never trust external input blindly.

### 4.4.7 `_publish_telemetry_frame` (lines 318–348)

Builds a **single JSON telemetry packet** (position, velocity, armed, mode,
battery, etc.) and publishes it as a `String` on `/drone_N/telemetry_frame`. This
mirrors flyMe's `TelemetryFrame` schema. It uses JSON-in-a-String **on purpose**
to avoid forcing every downstream consumer to rebuild a custom message type.

### 4.4.8 Heartbeat watchdog (`_on_orchestrator_heartbeat`, `_check_orchestrator_health`, lines 350–379)

A critical **failsafe**. The orchestrator (BehaviorManager) publishes a heartbeat
on `/swarm/orchestrator_heartbeat` at 2 Hz. DigiRC tracks the last time it heard
one. The 1 Hz timer checks:

```python
age = time.time() - self._last_heartbeat_ts
if age < self._heartbeat_timeout_s:        # default 3.0s
    return                                  # all good
# else: the brain is silent → autonomously LAND every drone
self._auto_land_triggered = True
for drone_id in list(self.drone_instances.keys()):
    asyncio.run_coroutine_threadsafe(self.land_drone(drone_id - 1), self.loop)
```

> 🧠 **Why this matters:** if the high-level brain dies while drones are armed and
> flying their last setpoint, they'd keep going blindly. The watchdog ensures they
> **land safely** instead. Note the threadsafe scheduling — the timer runs on the
> ROS thread but landing is async MAVSDK work.

### 4.4.9 `handle_command` (lines 381–410) — the service entry point

This is what runs when anyone calls `/drone_N/mission_command`:

```python
def handle_command(self, request, response, drone_id):
    valid_commands = ["ARM", "DISARM", "TAKEOFF", "GOTO", "LAND", "RTL", "OFFBOARD"]
    if request.command not in valid_commands:
        response.response = "invalid_command"; return response       # validation
    if drone_id not in self.drone_instances:
        response.response = "drone_not_connected"; return response   # connection check
    if self.loop:
        asyncio.run_coroutine_threadsafe(                            # schedule async work
            self.execute_drone_command(drone_id, request.command,
                                       request.x, request.y, request.z, request.yaw),
            self.loop)
    else:
        response.response = "event_loop_error"; return response
    response.response = "executing"; return response                 # reply fast
```

Three responsibilities: **validate**, **check connectivity**, **schedule + reply
"executing."** It never blocks.

### 4.4.10 `execute_drone_command` (lines 412–458) — the dispatcher

Runs on the event loop; routes the command string to the right coroutine:

```python
if   command == "ARM":      await self.arm_drone(drone_index)
elif command == "TAKEOFF":  await self.takeoff_drone(drone_index, z)
elif command == "GOTO":     await self.goto_drone(drone_index, x, y, z, yaw)
elif command == "LAND":     await self.land_drone(drone_index)
elif command == "OFFBOARD": await self.resume_offboard(drone_index)
elif command == "DISARM":   await drone.action.disarm()              # Phase B3
elif command == "RTL":      await drone.action.return_to_launch()    # Phase B3 (LAND fallback)
```

> 📝 **DISARM is real now.** The old doc said "no DISARM; use LAND." The current
> code **does** implement DISARM (motors stop immediately — dangerous; gated by the
> flyMe layer with `EMERGENCY_DISARM_ENABLED`). And **RTL** flies home, falling
> back to LAND if RTL fails.

### 4.4.11 `arm_drone` (lines 460–530) — arming + OFFBOARD

The canonical "do a drone thing" pattern. Steps:
1. **Wait until armable** (GPS/sensors OK), with a 30 s timeout.
2. **Send 5 setpoints** `(0,0,0,0)` *before* starting OFFBOARD — PX4 needs a queued
   setpoint stream first (the OFFBOARD rule from §2.3).
3. **Arm**, then **start OFFBOARD with up to 5 retries** (PX4 can reject the first
   attempt right after arming).
4. **Publish `armed = True`** on the topic.

```python
for _ in range(5):                                   # prime the setpoint stream
    await drone.offboard.set_position_ned(PositionNedYaw(0,0,0,0))
    await asyncio.sleep(0.1)
await drone.action.arm()
for attempt in range(5):                             # retry OFFBOARD start
    try:
        await drone.offboard.start(); break
    except OffboardError:
        await drone.offboard.set_position_ned(PositionNedYaw(0,0,0,0))
        await asyncio.sleep(0.3)
```

> **What breaks if you skip the 5 priming setpoints?** `offboard.start()` throws
> `OffboardError` ("no setpoint set") and arming-with-offboard fails. This is the
> #1 OFFBOARD gotcha.

### 4.4.12 `takeoff_drone` (lines 532–554) — controlled ascent

OFFBOARD is already active (from `arm`). Ascend smoothly from ground to target by
stepping the NED `down` setpoint over 51 steps (~5 s), then hold 20 steps (~2 s):

```python
target_altitude = -altitude                          # NED: up is negative
for i in range(51):
    z = (target_altitude / 50) * i                   # 0 → target, gradually
    await drone.offboard.set_position_ned(PositionNedYaw(0,0,z,0))
    await asyncio.sleep(0.1)
```

> Smooth interpolation avoids a violent jump that could destabilize the drone.

### 4.4.13 `_compute_ned_offsets` (lines 556–619) — the frame math

Each drone's PX4 treats *its own* takeoff spot as `(0,0)`. To make all drones share
one coordinate frame, DigiRC computes each drone's offset from a common origin:
- If `WORLD_ORIGIN_LAT`/`WORLD_ORIGIN_LON` env vars are set → use that as origin.
- Else → use drone 1's GPS home as the reference.

```python
dlat = drone_lat - ref_lat
dlon = drone_lon - ref_lon
north_offset = dlat * 111320.0                                   # ~m per degree lat
east_offset  = dlon * 111320.0 * math.cos(math.radians(ref_lat)) # lon shrinks with lat
self.drone_ned_offsets[drone_id] = (north_offset, east_offset)
```

> This "flat-earth" approximation is fine for the small areas a swarm covers.
> Without it, two drones told to "go to (10,0)" would fly to two *different* real
> places, because each measured from its own home. (Full explanation in §5.3.)

### 4.4.14 `goto_drone` (lines 621–666) — the real GOTO

> ⚠️ **This is the GOTO that the service actually uses** — *not* the 101-step loop
> in the deprecated `run_drone_mission`. It sends a **single setpoint** and returns.

```python
ned_offset   = self.drone_ned_offsets.get(drone_id, (0.0, 0.0))
target_north = x - ned_offset[0]     # global → this drone's local frame
target_east  = y - ned_offset[1]
target_down  = -z                    # altitude (up) → NED down
target_yaw_deg = math.degrees(yaw)   # MAVSDK wants degrees

if not self._offboard_started.get(drone_id, False):   # make sure OFFBOARD is on
    await drone.offboard.set_position_ned(PositionNedYaw(target_north, target_east, target_down, target_yaw_deg))
    await drone.offboard.start()
    self._offboard_started[drone_id] = True

await drone.offboard.set_position_ned(PositionNedYaw(target_north, target_east, target_down, target_yaw_deg))
```

> 🧠 **Why a single setpoint, not a flight loop?** DigiRC is a *setpoint relay*. The
> **continuous streaming** (re-sending the target many times per second to satisfy
> OFFBOARD) is done by the **layer above** (the orchestrator's 5 Hz loop, or a
> mission script). This keeps DigiRC simple and lets the smart planning live
> upstairs. PX4's own trajectory smoother flies the drone to the setpoint.

### 4.4.15 `land_drone` (lines 668–690) — land & disarm

```python
await drone.action.land()
async for state in drone.telemetry.landed_state():    # wait until on ground
    if state == LandedState.ON_GROUND: break
await drone.offboard.stop()
await drone.action.disarm()
self._offboard_started[drone_id] = False              # reset for next flight
```

### 4.4.16 `resume_offboard` (lines 692–726) — re-take a hovering drone

If a previous mission left a drone airborne in POSCTL (e.g. its LAND failed), a new
mission calls `OFFBOARD` to reclaim control: read current position, send several
hold setpoints there, then `offboard.start()`. Prevents a jump when control resumes.

### 4.4.17 `connect_all_drones` (lines 728–863) — bring drones online

For each drone:
- **Real mode:** `System()` then `connect(system_address="serial:///dev/ttyUSB0:57600")`.
- **Sim mode:** `System(mavsdk_server_address="127.0.0.1", port=grpc_port)` then
  `connect(system_address=f"udp://:{udp_port}")`.
- Wait for connection + position health (30 s timeouts).
- **Limit speed** for smooth control: `MPC_XY_VEL_MAX=2.5`, `MPC_XY_CRUISE=2.0`.
- **Record GPS home** for the frame math.
- After all connected → `self._compute_ned_offsets()` and start one
  **telemetry task per drone**.

### 4.4.18 Telemetry publishing (lines 865–998)

`telemetry_publisher_task(drone_id)` is an infinite async loop at **10 Hz** calling
`get_telemetry_data`, which publishes armed status + position + velocity +
telemetry_frame. Position is converted **local NED → global** (add offset) and
**altitude sign flipped** (`pos.z = -down`):

```python
pos.x = posvel.position.north_m + ned_offset[0]
pos.y = posvel.position.east_m  + ned_offset[1]
pos.z = -posvel.position.down_m            # NED down → positive-up altitude
```

> Note: errors in telemetry are swallowed (logged ~1% of the time) so a transient
> read failure never stops the stream. Robustness over noise.

### 4.4.19 `start_mavsdk_servers` (lines 1000–1036) — sim plumbing

Finds the `mavsdk_server` binary, then spawns **one per drone**:

```python
subprocess.Popen([mavsdk_server_path, "-p", str(port), f"udp://:{self.udp_ports[i]}"], ...)
```

Each server bridges one drone's MAVLink-UDP (`14541+i`) to MAVSDK-gRPC (`50041+i`).
Real mode skips this (MAVSDK connects to serial directly).

### 4.4.20 `main` (lines 1086–1130) — startup orchestration

```python
rclpy.init(args=args)
node = SwarmController()
executor = MultiThreadedExecutor(); executor.add_node(node)
threading.Thread(target=spin_ros, args=(executor,), daemon=True).start()   # ROS in bg
loop = asyncio.new_event_loop(); asyncio.set_event_loop(loop); node.loop = loop
async def run_receiver():
    await asyncio.sleep(2)                  # let mavsdk_servers boot
    await node.connect_all_drones()         # connect everything
    while rclpy.ok(): await asyncio.sleep(0.1)   # stay alive, serve commands
loop.run_until_complete(run_receiver())
```

This wires together both worlds: ROS callbacks on the executor thread, MAVSDK on
the asyncio loop, with `node.loop` as the bridge handle.

---

# 5. End-to-End Data Flow

## 5.1 Command flow (input → motors)

```
   ┌──────────────┐
   │ You / flyMe  │  "ARM drone 1"
   │ /orchestrator│
   └──────┬───────┘
          │ ros2 service call /drone_1/mission_command {command:"ARM"}
          ▼
   ┌──────────────────────────┐
   │ handle_command()  (ROS)  │  validate → check connected → schedule → "executing"
   └──────┬───────────────────┘
          │ asyncio.run_coroutine_threadsafe(...)
          ▼
   ┌──────────────────────────┐
   │ execute_drone_command()  │  dispatch by string
   └──────┬───────────────────┘
          ▼
   ┌──────────────────────────┐
   │ arm_drone()  (async)     │  wait armable → prime setpoints → arm → offboard.start
   └──────┬───────────────────┘
          │ MAVSDK: drone.action.arm()
          ▼
   ┌──────────────┐   gRPC :50041   ┌───────────────┐  MAVLink :14541  ┌──────────┐
   │  MAVSDK       │───────────────►│ mavsdk_server │────────────────►│   PX4    │──► motors
   │  System obj   │                │  (sim only)   │                  │ autopilot│
   └──────────────┘                 └───────────────┘                  └──────────┘
```

(On **real** hardware the `mavsdk_server` box disappears and MAVSDK talks straight
to the Pixhawk over `serial:///dev/ttyUSB0:57600`.)

## 5.2 Telemetry flow (sensors → you)

```
   ┌──────────┐ position estimate (EKF)
   │   PX4    │
   └────┬─────┘
        │ MAVLink
        ▼
   ┌───────────────┐  gRPC   ┌──────────────┐
   │ mavsdk_server │────────►│ MAVSDK System│  drone.telemetry.position_velocity_ned()
   └───────────────┘         └──────┬───────┘
                                    │ async stream
                                    ▼
                       ┌──────────────────────────┐
                       │ _get_position()           │  local NED → global (+offset),
                       │ (in 10 Hz telemetry task) │  z = -down (flip altitude)
                       └──────┬────────────────────┘
                              │ publish
                              ▼
            /drone_1/position (Point) , /drone_1/velocity (Vector3) ,
            /drone_1/armed (Bool) , /drone_1/telemetry_frame (String JSON)
                              │
                              ▼
                  orchestrator / flyMe / `ros2 topic echo`
```

## 5.3 The coordinate-frame conversion (why it exists)

```
Two drones spawned 5 m apart. Each PX4 says "I am at my own (0,0)".

  Without frame fix:                With DigiRC's offset fix:
  GOTO(10,0) sent to both           GOTO(10,0) = ONE world point
  → drone A flies to A+10           → both fly to the SAME (10,0)
  → drone B flies to B+10           → formations stay correct
  (they diverge!)                   (they agree!)

  goto_drone:   local = global − offset      (command path)
  _get_position: global = local + offset     (telemetry path)
```

`offset` is computed once in `_compute_ned_offsets()` from each drone's GPS home
versus the shared world origin.

---

# 6. Step-by-Step Rebuild Guide

You learn a system best by **rebuilding it**. Here we construct DigiRC from an
empty folder, in stages. Each stage: **Objective → Theory → Implementation →
Test → Expected output → Debug → Common mistakes.** (Do this *after* §7 setup.)

> Build in a scratch package (e.g. `my_digirc`) so you don't disturb the real one.

## Stage 0 — Create the package

**Objective:** an empty, buildable ROS 2 package.
**Theory:** a ROS 2 package needs `package.xml` + a build file. We use `ament_cmake`
because we'll define a service.
**Implementation:**
```bash
cd ~/project/v2_swarm/src
ros2 pkg create --build-type ament_cmake my_digirc --dependencies rclpy std_msgs geometry_msgs
```
**Test:** `cd ~/project/v2_swarm && colcon build --packages-select my_digirc`
**Expected:** `Finished <<< my_digirc`.
**Common mistake:** forgetting `source /opt/ros/humble/setup.bash` first → `ros2:
command not found`.

## Stage 1 — Define the command service

**Objective:** a `Command` service type.
**Theory:** services need a `.srv` with request/response split by `---`.
**Implementation:** create `srv/Command.srv`:
```
string command
float32 x
float32 y
float32 z
float32 yaw
---
string response
```
Add to `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Command.srv" DEPENDENCIES std_msgs)
```
and to `package.xml`:
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
**Test:** `colcon build` then `ros2 interface show my_digirc/srv/Command`.
**Expected:** prints your fields.
**Common mistake:** missing `DEPENDENCIES std_msgs` → build error about message deps.

## Stage 2 — A node that just exposes the service

**Objective:** a node that accepts commands and replies "executing" (no drone yet).
**Theory:** §2.1 services + §3.4 callbacks.
**Implementation (`src/my_node.py`):**
```python
import rclpy
from rclpy.node import Node
from my_digirc.srv import Command

class MiniController(Node):
    def __init__(self):
        super().__init__("mini_controller")
        self.create_service(Command, "drone_1/mission_command", self.handle)
    def handle(self, req, resp):
        valid = ["ARM","TAKEOFF","GOTO","LAND"]
        resp.response = "executing" if req.command in valid else "invalid_command"
        self.get_logger().info(f"got {req.command} → {resp.response}")
        return resp

def main():
    rclpy.init(); rclpy.spin(MiniController()); rclpy.shutdown()
if __name__ == "__main__": main()
```
Install it in `CMakeLists.txt`: `install(PROGRAMS src/my_node.py DESTINATION lib/${PROJECT_NAME})`.
**Test:**
```bash
colcon build && source install/setup.bash
ros2 run my_digirc my_node.py        # terminal 1
ros2 service call /drone_1/mission_command my_digirc/srv/Command "{command: 'ARM'}"  # terminal 2
```
**Expected:** terminal 2 prints `response='executing'`; terminal 1 logs the command.
**Common mistake:** forgetting to re-`source install/setup.bash` after build → "service not found."

## Stage 3 — Connect to one simulated drone (MAVSDK)

**Objective:** connect to a SITL drone and read its position.
**Theory:** §2.3 SITL, §3.9 asyncio, §12 MAVSDK.
**Implementation:** add async connect logic (mirror `connect_all_drones`):
```python
from mavsdk import System
async def connect(self):
    self.drone = System(mavsdk_server_address="127.0.0.1", port=50041)
    await self.drone.connect(system_address="udp://:14541")
    async for s in self.drone.core.connection_state():
        if s.is_connected: break
    self.get_logger().info("connected!")
```
Run a SITL first (see §13). **Test:** node logs "connected!".
**Common mistake:** no `mavsdk_server` running on 50041 → connection hangs. Start
the server (or let your node spawn it like `start_mavsdk_servers`).

## Stage 4 — Implement ARM + OFFBOARD

**Objective:** arm and hold at ground level.
**Theory:** §2.3 OFFBOARD rule; §4.4.11.
**Implementation:** copy the arm pattern: wait armable → 5 priming setpoints → arm →
`offboard.start()` with retries → publish armed.
**Test:** call ARM; watch Gazebo — props spin; `ros2 topic echo /drone_1/armed` → `data: true`.
**Common mistakes:** skipping priming setpoints → `OffboardError`; not awaiting
`is_armable` → "command denied."

## Stage 5 — TAKEOFF, GOTO, LAND

**Objective:** full flight cycle.
**Theory:** §4.4.12/14/15. Remember GOTO sends a single setpoint — for it to *hold*,
re-send it in a loop (Stage 6) or just test that the drone starts moving.
**Test:** ARM → TAKEOFF z=5 → GOTO x=10 → LAND. Watch `ros2 topic echo /drone_1/position`.
**Common mistake:** expecting GOTO to fly a full path on its own — it relies on a
setpoint stream; a single setpoint moves it toward the target but you need
continuous setpoints for robust tracking.

## Stage 6 — Telemetry loop + multi-drone + safety

**Objective:** match the real DigiRC.
**Implementation:**
- Wrap everything in `for i in range(num_drones)` (per-drone services/publishers).
- Add the 10 Hz `telemetry_publisher_task`.
- Add the heartbeat watchdog (§4.4.8).
- Add the NED frame offsets (§4.4.13).
**Test:** run with `NUM_DRONES=3`, command each drone independently, kill the
heartbeat publisher and confirm auto-LAND.

By Stage 6 you've rebuilt DigiRC. Now you understand every design decision because
you hit the problem each one solves.

---

# 7. Environment Setup

Complete, command-by-command. Target OS: **Ubuntu 22.04** (required for ROS 2
Humble).

## 7.1 Ubuntu

Install Ubuntu 22.04 LTS (native or VM). ROS 2 Humble officially targets 22.04.
*Why 22.04:* each ROS 2 release pins to one Ubuntu LTS; Humble ↔ Jammy (22.04).

## 7.2 ROS 2 Humble

```bash
# 1. Enable the Universe repo
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# 2. Add the ROS 2 apt key + repo
sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Install the desktop bundle (includes rqt, demos, etc.)
sudo apt update && sudo apt install -y ros-humble-desktop ros-dev-tools
```
**Each step:** (1) ROS packages live in Universe; (2) the key/repo let apt trust &
find them; (3) `ros-humble-desktop` is the full install, `ros-dev-tools` adds
`colcon` and build helpers.

**Source it (every new terminal):**
```bash
source /opt/ros/humble/setup.bash
# Optional convenience — auto-source on new shells:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 7.3 PX4 (SITL)

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh        # installs all PX4 + sim dependencies
# (log out / reboot if prompted, then:)
make px4_sitl gazebo-classic         # first build of SITL (slow the first time)
```
*Why:* this gives you the PX4 firmware + the Gazebo-classic integration the launch
scripts expect. (`~/PX4-Autopilot` is exactly where this project's scripts look —
see `PX4_DIR` in `launch_nav_sim_cam.sh`.)

## 7.4 Gazebo

`make px4_sitl gazebo-classic` pulls in **Gazebo Classic** automatically. To verify:
```bash
gazebo --version
```
*Why Gazebo Classic:* the PX4 SITL models and this project's worlds
(`nav_obstacles.world`, the `iris`/`iris_cam` models) target Gazebo Classic.

## 7.5 MAVSDK (Python) + the server binary

```bash
pip3 install mavsdk          # the Python SDK DigiRC imports
```
The **`mavsdk_server`** binary is already committed in the repo root
(`v2_swarm/mavsdk_server`) — `start_mavsdk_servers()` searches for it. If you need a
fresh one:
```bash
wget https://github.com/mavlink/MAVSDK/releases/download/v1.4.13/mavsdk_server_linux_x64
chmod +x mavsdk_server_linux_x64 && mv mavsdk_server_linux_x64 ~/project/v2_swarm/mavsdk_server
```

> ❗ **There is NO MAVROS to install.** If a tutorial says
> `sudo apt install ros-humble-mavros`, you do **not** need it for DigiRC.

## 7.6 Build the workspace + environment variables

```bash
cd ~/project/v2_swarm
colcon build --symlink-install      # build all packages (digi_rc, swarm_orchestrator)
source install/setup.bash           # make them importable
```

**Environment variables DigiRC reads** (set before launching):

| Variable | Default | Meaning |
|---|---|---|
| `DRONE_MODE` | `simulation` | `simulation` or `real`. |
| `NUM_DRONES` | `5` | Number of simulated drones. |
| `DRONE_CONNECTIONS` | *(empty)* | Comma-separated serial URIs for real drones; count sets `num_drones`. |
| `MISSION_SPEED` | `1.0` | Speed multiplier (0.1–5.0). |
| `ORCHESTRATOR_HEARTBEAT_ENABLED` | `true` | Enable the auto-LAND watchdog. |
| `ORCHESTRATOR_HEARTBEAT_TIMEOUT` | `3.0` | Seconds of silence before auto-LAND. |
| `WORLD_ORIGIN_LAT` / `WORLD_ORIGIN_LON` | *(unset)* | Shared world origin for the NED frame; else drone 1 is origin. |

## 7.7 Verification checklist

```bash
ros2 doctor                              # general ROS health
ros2 interface show digi_rc/srv/Command  # service type exists → build worked
python3 -c "import mavsdk; print('mavsdk ok')"
ls -l ~/project/v2_swarm/mavsdk_server   # the server binary is present + executable
ls ~/PX4-Autopilot                       # PX4 is where scripts expect
```

---

# 8. Creating Your First ROS 2 Node

A focused tutorial in the *DigiRC style* (rclpy, services, topics).

## 8.1 Create the package
```bash
cd ~/project/v2_swarm/src
ros2 pkg create --build-type ament_python hello_drone --dependencies rclpy std_msgs geometry_msgs
```
*(`ament_python` here because this toy has no `.srv`; DigiRC uses `ament_cmake`
because it does.)*

## 8.2 A publisher node (mimics DigiRC telemetry)
`hello_drone/hello_drone/talker.py`:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class FakeTelemetry(Node):
    def __init__(self):
        super().__init__("fake_telemetry")
        self.pub = self.create_publisher(Point, "/drone_1/position", 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)      # 10 Hz, like DigiRC
    def tick(self):
        p = Point(); p.x = self.t; p.y = 0.0; p.z = 5.0
        self.pub.publish(p); self.t += 0.1

def main():
    rclpy.init(); rclpy.spin(FakeTelemetry()); rclpy.shutdown()
```

## 8.3 A subscriber node
```python
def cb(msg): print(f"drone at x={msg.x:.1f}")
node.create_subscription(Point, "/drone_1/position", cb, 10)
```

## 8.4 Test with the CLI
```bash
ros2 run hello_drone talker            # terminal 1
ros2 topic echo /drone_1/position      # terminal 2 — see the stream
ros2 topic hz /drone_1/position        # confirm ~10 Hz
rqt_graph                              # see the node + topic visually
```

You've now reproduced DigiRC's telemetry pattern in miniature.

---

# 9. Building a Basic Drone Controller

Now the real thing: connect to a SITL drone and fly it (mirrors DigiRC internals).

## 9.1 Connect (MAVSDK)
```python
from mavsdk import System
drone = System(mavsdk_server_address="127.0.0.1", port=50041)
await drone.connect(system_address="udp://:14541")
async for s in drone.core.connection_state():
    if s.is_connected: break
```

## 9.2 Verify it's healthy (pre-arm check)
```python
async for health in drone.telemetry.health():
    if health.is_global_position_ok and health.is_home_position_ok:
        break       # GPS + home set → safe to operate
```
*Why:* PX4 refuses to arm without a position estimate. DigiRC waits for exactly
this in `connect_all_drones`/`arm_drone`.

## 9.3 Arm
```python
await drone.action.arm()
```

## 9.4 Enter OFFBOARD (the right way)
```python
from mavsdk.offboard import PositionNedYaw, OffboardError
for _ in range(5):                                   # prime setpoints (mandatory!)
    await drone.offboard.set_position_ned(PositionNedYaw(0,0,0,0))
    await asyncio.sleep(0.1)
await drone.offboard.start()
```

## 9.5 Takeoff (ascend in NED)
```python
for i in range(51):
    await drone.offboard.set_position_ned(PositionNedYaw(0,0,(-5.0/50)*i,0))  # to 5 m up
    await asyncio.sleep(0.1)
```

## 9.6 Send a position (GOTO)
```python
await drone.offboard.set_position_ned(PositionNedYaw(10.0, 5.0, -5.0, 0.0))  # 10N,5E,5m up
```

## 9.7 Land
```python
await drone.action.land()
```

## 9.8 "RC override" basics
Classic RC control sends raw stick channels. PX4/MAVSDK exposes
`drone.manual_control` and `set_actuator` for that, but **DigiRC deliberately uses
position setpoints (OFFBOARD) instead of raw RC override** — it's safer and
higher-level. (Mentioned because the name "DigiRC" evokes RC channels, but the
implementation is setpoint-based.)

**Safety notes:** always test in SITL first; in real flight keep a physical RC
transmitter with a kill switch ready; arm only in a clear, open area.

---

# 10. Building DigiRC Core

This section frames the **architecture** of the real `SwarmController` as a set of
layers you can build and test independently.

```
┌──────────────────────────────────────────────────────────────┐
│ 1. ROS Interface Layer    services + publishers + subscribers │  ← §4.4.5
├──────────────────────────────────────────────────────────────┤
│ 2. Command Handler        handle_command → execute_drone_command │ ← §4.4.9/10
├──────────────────────────────────────────────────────────────┤
│ 3. Control Loop / Actions arm/takeoff/goto/land/rtl coroutines │  ← §4.4.11–16
├──────────────────────────────────────────────────────────────┤
│ 4. Telemetry / State      10 Hz tasks, position/velocity/armed │  ← §4.4.18
├──────────────────────────────────────────────────────────────┤
│ 5. Failsafe Layer         heartbeat watchdog → auto-LAND       │  ← §4.4.8
├──────────────────────────────────────────────────────────────┤
│ 6. Frame Manager          NED offsets, global↔local            │  ← §4.4.13
├──────────────────────────────────────────────────────────────┤
│ 7. Connection Manager     connect_all_drones, mavsdk_server    │  ← §4.4.17/19
└──────────────────────────────────────────────────────────────┘
```

**For each layer:**
- **Architecture:** what it owns (see the section refs above).
- **Implementation:** the code is already in `multi_drone.py` — read those lines.
- **Testing:** in §6 stages and §14 SOP.
- **Debugging:** in §15 by symptom.

> 🧠 **The defining design choice:** DigiRC is a **thin, stateless-ish relay** —
> it converts commands ↔ MAVSDK and streams telemetry, but it does **not** plan,
> avoid obstacles, or hold formations. That intelligence lives *above* it
> (swarm_orchestrator). This separation is why the same DigiRC works for circle
> formations, navigation, SAR missions, and ML avoidance without changes.

---

# 11. ROS Topics & Services Deep Dive

## 11.1 Service: `/drone_N/mission_command`

| Property | Value |
|---|---|
| Type | `digi_rc/srv/Command` |
| Server | `SwarmController.handle_command` |
| Clients | orchestrator (`MissionExecutor`), flyMe (subprocess), you (CLI) |
| Request | `command:str, x,y,z:float32, yaw:float32` |
| Response | `response:str` (`executing`/`invalid_command`/`drone_not_connected`/`event_loop_error`) |

Inspect & call:
```bash
ros2 service list | grep mission_command
ros2 service type /drone_1/mission_command
ros2 service call /drone_1/mission_command digi_rc/srv/Command \
  "{command: 'TAKEOFF', x: 0.0, y: 0.0, z: 5.0, yaw: 0.0}"
```

## 11.2 Published topics

| Topic | Type | Rate | Publisher | Meaning |
|---|---|---|---|---|
| `/drone_N/position` | `geometry_msgs/Point` | 10 Hz | `_get_position` | global NED, **+up altitude** |
| `/drone_N/velocity` | `geometry_msgs/Vector3` | 10 Hz | `_get_position` | N/E/D velocity m/s |
| `/drone_N/armed` | `std_msgs/Bool` | 10 Hz | `_get_armed_status` | motors armed? |
| `/drone_N/telemetry_frame` | `std_msgs/String` (JSON) | 10 Hz | `_publish_telemetry_frame` | unified frame |
| `/swarm_status` | `std_msgs/String` | event | mission complete etc. | swarm-level status |

## 11.3 Subscribed topics

| Topic | Type | Subscriber | Meaning |
|---|---|---|---|
| `/drone_N/speed_command` | `std_msgs/Float32` | `speed_callback` | live speed multiplier (clamped 0.1–5) |
| `/swarm/orchestrator_heartbeat` | `std_msgs/String` | `_on_orchestrator_heartbeat` | proof the brain is alive |

## 11.4 Inspecting, echoing, visualizing

```bash
ros2 topic list                              # all channels
ros2 topic info /drone_1/position            # type + pub/sub counts
ros2 topic echo /drone_1/position            # watch live values
ros2 topic hz /drone_1/position              # measure actual rate
ros2 topic echo /drone_1/telemetry_frame     # see the JSON frame
rqt_graph                                    # visual node/topic graph
ros2 run rqt_plot rqt_plot /drone_1/position/z   # live plot altitude
```

---

# 12. MAVSDK Deep Dive

DigiRC's control library. A MAVSDK **`System`** object represents one drone; its
sub-modules group functionality.

## 12.1 The modules DigiRC uses

| MAVSDK module | Calls used in DigiRC | Purpose |
|---|---|---|
| `core` | `connection_state()` | detect connect/disconnect |
| `telemetry` | `health()`, `armed()`, `position_velocity_ned()`, `landed_state()`, `home()` | read state |
| `action` | `arm()`, `disarm()`, `land()`, `return_to_launch()` | discrete commands |
| `offboard` | `set_position_ned()`, `start()`, `stop()` | continuous position control |
| `param` | `set_param_float("MPC_XY_VEL_MAX", ...)` | tune PX4 params |

## 12.2 Connection strings

| Form | Used for |
|---|---|
| `udp://:14541` | simulation (via mavsdk_server) |
| `serial:///dev/ttyUSB0:57600` | real telemetry radio |
| `serial:///dev/ttyACM0:57600` | real direct USB |
| `tcp://192.168.1.10:5760` | network link |

## 12.3 Setpoint requirements (the OFFBOARD contract, restated)

1. You **must** send at least one setpoint **before** `offboard.start()`.
2. After starting, you **must** keep sending setpoints (≥2 Hz) or PX4 exits OFFBOARD.
3. `set_position_ned` uses **NED**: `down` is negative for "up."

DigiRC honors #1 (5 priming setpoints in `arm_drone`/`goto_drone`) and delegates #2
to the streaming layer above it.

## 12.4 If you used MAVROS instead (comparison)

| Task | MAVSDK (DigiRC) | MAVROS equivalent |
|---|---|---|
| Arm | `await drone.action.arm()` | call service `/mavros/cmd/arming` |
| Set position | `await drone.offboard.set_position_ned(...)` | publish to `/mavros/setpoint_position/local` |
| Read position | `async for p in drone.telemetry.position_velocity_ned()` | subscribe `/mavros/local_position/pose` |
| Set mode | `await drone.offboard.start()` | call `/mavros/set_mode` |

Same drone, two philosophies. DigiRC picked MAVSDK for the explicit per-drone
`async` style.

## 12.5 Common PX4/MAVSDK issues (preview of §15)
- `OffboardError: NO_SETPOINT_SET` → you forgot to prime setpoints.
- Arming denied → no GPS/home, or a failsafe is active.
- Connection hangs → wrong port, or `mavsdk_server` not running.

---

# 13. Gazebo Testing Workflow

## 13.1 Launch simulation (the project way)
```bash
cd ~/project/v2_swarm
# Easiest: the project launch script (starts Gazebo + PX4 + DigiRC):
./launch_gazebo_for_flyme.sh
# Or camera + avoidance variant:
NUM_DRONES=3 ./launch_nav_sim_cam.sh
```

## 13.2 Launch SITL manually (to understand the pieces)
```bash
# Terminal A — PX4 + Gazebo (single drone):
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic
# This opens Gazebo with an `iris` drone; PX4 listens on udp://:14540 (offboard) and
# the project maps additional drones to 14541+.
```

## 13.3 Spawning multiple drones
The project uses PX4's `Tools/simulation/gazebo-classic/sitl_multiple_run.sh`
(wrapped by the launch scripts) with a spawn argument like
`iris_cam:1:X:Y` per drone. Each instance gets its own UDP port; DigiRC's
`udp_ports = [14541+i]` matches them.

## 13.4 Verify the stack is alive
```bash
pgrep -x gzserver        # Gazebo physics running?
pgrep -f mavsdk_server   # one per drone?
ros2 node list           # swarm_controller_node present?
ros2 service list | grep mission_command   # one service per drone?
ros2 topic echo /drone_1/armed             # telemetry flowing?
```

## 13.5 Test commands
```bash
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'ARM',  x:0.0,y:0.0,z:0.0,yaw:0.0}"
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'TAKEOFF',x:0.0,y:0.0,z:5.0,yaw:0.0}"
# Watch the drone rise in the Gazebo window and:
ros2 topic echo /drone_1/position
```

---

# 14. Live Arm & Flight Testing SOP

A **Standard Operating Procedure**. Follow in order. Console expectations included.

### Pre-flight checks
```bash
ros2 node list | grep swarm_controller_node     # expect: /swarm_controller_node
ros2 service list | grep mission_command        # expect: /drone_1/mission_command ...
ros2 topic hz /drone_1/position                 # expect: ~10 Hz
ros2 topic echo /drone_1/armed --once           # expect: data: false (not armed yet)
```

### Connection verification
DigiRC logs (in its terminal) should show, per drone:
```
Drone 1: Connected
Drone 1: Position estimate ready
Drone 1: Home GPS = (47.397..., 8.545..., 488.0m)
Drone 1: NED offset from reference = (N=0.00m, E=0.00m)
```

### Arm
```bash
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'ARM',x:0.0,y:0.0,z:0.0,yaw:0.0}"
```
Expect service reply `response='executing'`; DigiRC logs `Armed` then `Offboard
started - holding at ground level (z=0)`; `/drone_1/armed` flips to `true`.

### Takeoff → Hover
```bash
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'TAKEOFF',x:0.0,y:0.0,z:5.0,yaw:0.0}"
```
Expect `Taking off to 5.0m` → `Takeoff complete, holding at 5.0m`; `position.z → ~5.0`.

### Move
```bash
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'GOTO',x:10.0,y:0.0,z:5.0,yaw:0.0}"
```

### Land → Disarm
```bash
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'LAND',x:0.0,y:0.0,z:0.0,yaw:0.0}"
```
Expect `Landing` → `Landed and disarmed`; `/drone_1/armed` → `false`.

### Emergency
- **Software:** `ros2 service call ... {command:'RTL'}` (fly home) or `{command:'LAND'}`.
- **Heartbeat failsafe:** if the orchestrator dies, DigiRC auto-LANDs after 3 s.
- **Real flight:** physical RC kill switch is the final authority.

---

# 15. Debugging Handbook

Format per issue: **Symptoms → Root cause → Diagnose → Fix.**

## 15.1 MAVSDK/PX4 won't connect
**Symptoms:** DigiRC stuck at "Waiting for connection..."; no `Connected` log.
**Root cause:** `mavsdk_server` not running, wrong port, or SITL not up.
**Diagnose:**
```bash
pgrep -x gzserver ; pgrep -f mavsdk_server
ss -ulnp | grep 14541        # is the UDP port bound?
tail -f /tmp/nav_cam_gazebo.log
```
**Fix:** ensure SITL is running; confirm `udp_ports`/`grpc_ports` match the SITL
instances; verify `mavsdk_server` binary exists and is executable.

## 15.2 OFFBOARD rejected (`OffboardError`)
**Symptoms:** `Offboard start attempt N failed`; arm aborts.
**Root cause:** no setpoint queued before `offboard.start()`, or stream too slow.
**Diagnose:** check arm logs for the retry messages.
**Fix:** the priming loop (5 setpoints) must run before `start()`; if you wrote a
caller, send GOTO setpoints continuously (≥2 Hz).

## 15.3 Arming denied / "Not armable after timeout"
**Symptoms:** `Not armable after timeout`.
**Root cause:** GPS not locked, home not set, or a failsafe (low battery, RC loss).
**Diagnose:**
```bash
ros2 topic echo /drone_1/telemetry_frame      # check health fields
```
In SITL, give it more time after launch; on real drones, go outside for GPS lock.
**Fix:** wait for `is_global_position_ok && is_home_position_ok`; clear failsafes.

## 15.4 No telemetry on `/drone_N/position`
**Symptoms:** `ros2 topic echo` prints nothing.
**Root cause:** drone never connected (no telemetry task started), or wrong drone id.
**Diagnose:** `ros2 topic list | grep drone` ; check DigiRC logs for "Telemetry task
started for Drone N".
**Fix:** confirm the drone connected; check `NUM_DRONES` matches reality.

## 15.5 Wrong namespaces / service not found
**Symptoms:** `ros2 service call` → "service not available."
**Root cause:** forgot to `source install/setup.bash`, or drone id off-by-one
(drones are **1-indexed**).
**Fix:** source the workspace; use `/drone_1/...` not `/drone_0/...`.

## 15.6 Callback / threading bugs
**Symptoms:** all drones react to a command meant for one; or commands silently
ignored.
**Root cause:** missing `did=drone_id` capture in a lambda (§3.4); or `self.loop`
not set when a command arrives.
**Diagnose:** check `event_loop_error` responses (means `self.loop` was `None`).
**Fix:** keep the `did=` default-arg pattern; ensure `connect_all_drones` ran (loop
is created in `main` before serving).

## 15.7 Stale setpoints / drift
**Symptoms:** drone drifts or doesn't hold position.
**Root cause:** the streaming layer above DigiRC stopped sending setpoints; PX4
holds the last one and OFFBOARD may drop.
**Fix:** ensure the orchestrator/mission loop keeps publishing targets (DigiRC's
GOTO is single-shot by design — §4.4.14).

## 15.8 Drones fly to different places (frame bug)
**Symptoms:** formation distorted; "go to (10,0)" sends drones to different spots.
**Root cause:** NED offsets not computed (no GPS home), or wrong world origin.
**Diagnose:** look for `Reference drone has no home GPS!` warning; check the
`NED offset from reference` logs.
**Fix:** ensure each drone reports a GPS home; set `WORLD_ORIGIN_LAT/LON` if needed.

## 15.9 Simulation crashes / stale processes
**Symptoms:** Gazebo won't start; "address in use."
**Fix:** the launch scripts already `pkill` stale `px4/gzserver/gzclient/mavsdk_server`.
Manually:
```bash
pkill -9 px4 gzserver gzclient mavsdk_server
```

---

# 16. Codebase Engineering Standards

Patterns this codebase follows (and you should too).

- **Naming:** `snake_case` functions/vars; `PascalCase` classes; topics
  `/<namespace>/<thing>` (`/drone_1/position`); drones **1-indexed**.
- **Package layout:** one job per package; `srv/` for interfaces, `src/` for nodes;
  metadata in `package.xml`, build in `CMakeLists.txt`.
- **Logging:** use `self.get_logger().info/warn/error()`, **not** `print` in node
  code (it integrates with `/rosout` and ROS log levels). Throttle spam (DigiRC logs
  ~1% of telemetry errors).
- **Exception handling:** wrap every external/async call; a single drone's failure
  must not crash the swarm; clean up (disarm) on failure.
- **Validation at boundaries:** validate commands (`valid_commands`) and clamp
  external inputs (`speed_callback`).
- **Modularity / scalability:** per-drone resources built in loops, keyed by id;
  adding a drone is just `NUM_DRONES+1`.
- **Swarm-safe design:** failsafes are independent and layered (heartbeat watchdog,
  per-command validation, speed clamps).
- **Config over code:** behavior controlled by env vars, not edits.
- **Fire-and-forget where latency matters; await where confirmation matters.**

---

# 17. Swarm Architecture Relation

DigiRC is built for swarms from the ground up:

- **Per-drone namespacing:** every interface is `/drone_N/...`, so N drones are N
  independent, identical control surfaces.
- **Concurrent control:** the asyncio model lets one process command many drones
  without blocking (the orchestrator's `arm_all()` fires at all of them with
  `asyncio.gather`).
- **Shared coordinate frame:** the NED offset system (§5.3) is what makes
  *coordinated* behavior (formations) possible — without it each drone lives in its
  own frame.
- **Distributed control readiness:** because the interface is pure ROS 2, you can
  run controllers on separate machines and they auto-discover via DDS — no central
  master to bottleneck.
- **Telemetry aggregation:** higher layers subscribe to all `/drone_N/position`
  topics and assemble a swarm-wide picture; `/swarm_status` carries swarm-level
  events.

```
              swarm_orchestrator (one brain)
        ┌───────────┬───────────┬───────────┐
        ▼           ▼           ▼           ▼
   /drone_1/*   /drone_2/*   /drone_3/*  ... /drone_N/*     ← DigiRC interfaces
        │           │           │           │
     MAVSDK#1    MAVSDK#2    MAVSDK#3     MAVSDK#N           ← one System per drone
        │           │           │           │
      PX4#1       PX4#2       PX4#3       PX4#N
```

---

# 18. Advanced Improvements

Concrete directions to evolve DigiRC (good contribution targets):

- **Full HAL adoption:** the project defines `AbstractDroneHAL`
  (`swarm_orchestrator/hardware/abstract_hal.py`). Refactor DigiRC to *implement* it
  via `SitlHAL`/`PixhawkHAL` so sim/real differences are isolated behind one
  interface.
- **Better async hygiene:** replace broad `except: pass` telemetry swallowing with
  structured logging + health metrics; expose telemetry errors as a topic.
- **Setpoint streaming inside DigiRC (optional mode):** an opt-in continuous-GOTO
  mode so simple clients don't need their own 5 Hz loop.
- **QoS tuning:** use explicit QoS profiles (best-effort for high-rate telemetry,
  reliable for commands) instead of the default depth-10.
- **DDS configuration:** tune CycloneDDS (the project ships `cyclonedds_wifi.xml`)
  for multi-machine / Wi-Fi swarms.
- **Real-hardware integration:** add per-drone failsafe params, RC-loss handling,
  and battery-aware behavior at the DigiRC layer.
- **Interfaces:** joystick/gamepad bridge, a web dashboard (flyMe already does
  this), cloud telemetry sink.
- **Observability:** Prometheus metrics, structured JSON logs, a `/diagnostics`
  topic.

---

# 19. Developer Exercises

Hands-on tasks, roughly increasing in difficulty. Do them in a scratch branch.

1. **Add a `HOVER` command.** Extend `valid_commands`, add a `hover_drone` coroutine
   that reads current NED position and re-sends it for N seconds. Test in Gazebo.
2. **Add a new RC-style channel.** Subscribe to `/drone_N/yaw_command` (Float32) and
   feed it into the yaw of the next setpoint.
3. **Add a telemetry logger.** New node subscribing to all `/drone_N/position` and
   writing CSV. (Pattern in §8.)
4. **Add a battery failsafe.** Read battery from telemetry; if < 20%, auto-RTL that
   drone (mirror the heartbeat watchdog pattern in §4.4.8).
5. **Build a keyboard controller.** A client node mapping WASD keys → GOTO offsets
   from current position → service calls.
6. **Add a mission trigger.** A service that runs a scripted ARM→TAKEOFF→square→LAND
   sequence on one drone.
7. **Scale test.** Run `NUM_DRONES=5`, command all concurrently, and confirm the 10
   Hz telemetry holds (`ros2 topic hz`).

For each: write the **objective**, **test plan**, and **expected output** before
coding — exactly the discipline in §6.

---

# 20. Final Production Blueprint

Where DigiRC should head for real deployment.

## 20.1 What to refactor
- **Split the one file** into modules: `connection.py`, `commands.py`,
  `telemetry.py`, `failsafe.py`, `frames.py`. The single 1100-line file is fine for
  learning but hard to test/maintain.
- **Implement `AbstractDroneHAL`** so sim/real is a swap, not branching `if
  self.drone_mode == 'real'` throughout.
- **Add unit tests** for pure logic (frame math `_compute_ned_offsets`, command
  validation) and integration tests against SITL.
- **Remove dead code** (`MavsdkDrone`, `run_drone_mission`) once nothing references
  them.

## 20.2 Production architecture
```
┌────────────────────────────────────────────────────────────┐
│ Ground Station (flyMe + orchestrator)                        │
│   redundant, observable, authenticated command path          │
└───────────────┬──────────────────────────────────────────────┘
                │ secured DDS / VPN
┌───────────────▼──────────────────────────────────────────────┐
│ Per-drone companion computer (e.g. Raspberry Pi 5)            │
│   DigiRC (as HAL impl) + local failsafes + ML inference       │
│   runs even if the link to ground drops                       │
└───────────────┬──────────────────────────────────────────────┘
                │ serial / UART
┌───────────────▼──────────────────────────────────────────────┐
│ Pixhawk (PX4)  — hard real-time flight control                │
└────────────────────────────────────────────────────────────┘
```

## 20.3 Scaling considerations
- **Run DigiRC on each drone's companion computer**, not centrally — survives link
  loss, scales linearly.
- **QoS + DDS tuning** for lossy Wi-Fi/radio links.
- **Bandwidth:** at 10 Hz × (position+velocity+armed+frame) × N drones, telemetry
  adds up — consider rate control / compression for large swarms.
- **Time sync** across drones (PTP/NTP) for coordinated timed maneuvers.

## 20.4 Real-deployment concerns
- **Regulatory:** altitude/airspace limits, geofencing (the orchestrator's
  `SwarmGeofence` enforces this above DigiRC).
- **Safety:** independent hardware kill switch; battery & RC-loss failsafes in PX4
  *and* software; pre-arm checklists.
- **Security:** authenticate the command path; never expose `mission_command`
  services on an open network.
- **Observability:** every command, failsafe, and disarm logged with timestamps for
  incident review.

---

## 📎 Appendix: Quick command cheat-sheet

```bash
# Build & source (every new terminal):
cd ~/project/v2_swarm && colcon build --symlink-install && source install/setup.bash

# Launch sim (Gazebo + PX4 + DigiRC):
./launch_gazebo_for_flyme.sh

# Inspect:
ros2 node list ; ros2 topic list ; ros2 service list | grep mission_command
ros2 topic echo /drone_1/position ; ros2 topic hz /drone_1/position

# Fly drone 1:
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'ARM',x:0.0,y:0.0,z:0.0,yaw:0.0}"
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'TAKEOFF',x:0.0,y:0.0,z:5.0,yaw:0.0}"
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'GOTO',x:10.0,y:5.0,z:5.0,yaw:0.0}"
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command:'LAND',x:0.0,y:0.0,z:0.0,yaw:0.0}"

# Clean up stale sim processes:
pkill -9 px4 gzserver gzclient mavsdk_server
```

---

> **You've finished the handbook.** If you understood §4 (the code walkthrough),
> §5 (data flow), and §2.3's OFFBOARD rule, you can read, run, debug, and extend
> DigiRC. The single most important mental model: **DigiRC is a thin, per-drone
> translator between ROS 2 commands and MAVSDK — the intelligence lives above it,
> the autopilot lives below it.** Welcome to the swarm. 🛩️