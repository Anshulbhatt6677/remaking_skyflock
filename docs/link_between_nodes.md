# How the Swarm Nodes Communicate

Here is the big picture of how your 3 Python files work together to make the drones fly:

---

### 1. The Big Picture (How they communicate)

In ROS 2 (Robot Operating System), programs don't call each other directly like normal Python files. Instead, they act like independent "Nodes" on a network that pass messages to each other.

Here is the flow of communication we built:
1. **The Keyboard Controller** listens to your physical keyboard. When you press `w`, it shouts the word `"FORWARD"` onto a public channel (called a **Topic**). 
2. **The Orchestrator (The Brain)** is listening to that public channel. It hears `"FORWARD"`, does some math to figure out where the drones should move, and sends exact coordinate instructions to the low-level controller.
3. **The Swarm Controller / Manager (The Muscles)** receives those coordinates and translates them into raw MAVSDK commands that the PX4 simulation understands.

Let's look at exactly how we coded each step.

---

### Step 1: The Keyboard Controller (`swarm_keyboard_controller.py`)

**The Goal:** Read a single key press from your keyboard *without* requiring you to press the `Enter` key, and shout it to the network.

**The Code:**
```python
import sys
import select
import tty
import termios
```
These four libraries are built-in Python tools that talk directly to your computer's terminal:
* `sys` and `select` let us check if a key has been pressed without "blocking" or freezing the program.
* `tty` and `termios` let us put the terminal into **"raw mode"**. Normally, a terminal waits for you to type a whole sentence and press `Enter`. "Raw mode" forces it to instantly grab single keystrokes like `w` or `v`.

```python
moveBindings = {
    'w': 'FORWARD',
    's': 'BACKWARD',
    'v': 'V',
    # ... etc
}
```
We created a dictionary (a lookup table). If the script detects the `w` key, it looks it up in this table and finds the word `"FORWARD"`.

```python
self.publisher_ = self.create_publisher(String, '/swarm/command', 10)
...
self.publisher_.publish(msg)
```
This is the ROS 2 magic. We created a **Publisher**. Every time you press `w`, it packages the word `"FORWARD"` into a `String` message and publishes it to a topic called `/swarm/command`. It doesn't care who is listening; it just shouts it out.

---

### Step 2: The Orchestrator (`swarm_orchestrator.py`)

**The Goal:** Listen to `/swarm/command`, keep track of where the swarm currently is, and calculate the math for the formations.

**The Code:**
```python
# Swarm center and active shape state
self.center_x = 10.0
self.center_y = 0.0
self.center_z = -8.0
self.current_formation = None
```
When this script starts, it stores a "virtual center point" in its memory. Imagine this as an invisible dot floating in the Gazebo sky at `x=10, y=0, z=-8` (Z is negative because in drone coordinates, Up is negative!).

```python
self.create_subscription(String, "/swarm/command", self.command_callback, 10)
```
Here, the orchestrator sets up a **Subscription** to listen to the exact same `/swarm/command` topic that the keyboard controller is shouting on. Whenever it hears a message, it triggers the `command_callback` function.

```python
def command_callback(self, msg):
    cmd = msg.data.upper()
    ...
    elif cmd == "FORWARD":
        self.center_x += 1.0
        self.update_formation()
```
If the message it hears is `"FORWARD"`, it takes its invisible center dot (`self.center_x`) and adds `1.0` meter to it. It then calls `update_formation()`.

```python
def form_v(self, center_x, center_y, center_z, spacing=5.0):
    # Drone 1 (Leader): At center
    self.send_command(1, "GOTO", center_x, center_y, center_z)
    # Drone 2 (Left wing): Behind and Left
    self.send_command(2, "GOTO", center_x - spacing, center_y - spacing, center_z)
```
This is where the math happens. If the current shape is a V, the orchestrator says:
* "Drone 1, you go exactly to the invisible center dot."
* "Drone 2, you go 5 meters behind the center dot, and 5 meters to the left."
It takes these calculated coordinates and passes them down to the Swarm Controller.

---

### Step 3: The Manager (`swarm_manager.py`)

**The Goal:** Take the exact coordinates from the Orchestrator and safely stream them to the PX4 flight controller over MAVSDK.

**The Code:**
```python
def goto_position(self, drone_id, x, y, z, yaw):
    self.drones[drone_id]["target_position"] = PositionNedYaw(x, y, z, yaw)
```
When the orchestrator sends a coordinate, the manager simply saves it into a variable called `"target_position"`.

```python
async def _setpoint_loop(self, drone_id):
    while True:
        target = self.drones[drone_id].get("target_position")
        if target is not None:
            await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.1)
```
This is the **Continuous Streaming** loop we built earlier! It runs forever in the background. 10 times every second (`sleep(0.1)`), it looks at the `"target_position"` variable and forces the drone to fly there. 

Because it fires 10 times a second, the drone's autopilot stays locked in `OFFBOARD` mode and smoothly glides to the new position whenever you press `w`!
