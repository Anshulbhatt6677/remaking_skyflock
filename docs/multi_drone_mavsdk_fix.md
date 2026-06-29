# Multi-Drone MAVSDK Control Fix

This document explains why the initial implementation of the DIGIRC swarm controller failed to independently control multiple drones, and how the issue was resolved.

## The Problem

When trying to control multiple drones, both drones successfully took off, but independent commands (like landing only Drone 2) failed to affect the intended vehicle. The ROS2 nodes and MAVSDK code were executing without errors, and the network ports (14540, 14541) were correctly mapped to PX4 instances. 

The issue lies in how **MAVSDK-Python** manages connections. 

### MAVSDK Architecture
MAVSDK-Python is actually a wrapper. When you instantiate a `System()` object in Python, it doesn't directly talk to PX4. Instead, it starts a background C++ process called `mavsdk_server` and communicates with it via a **gRPC connection**.

1. **Python Client** (`System()`) --- *gRPC* ---> **mavsdk_server** (C++) --- *UDP* ---> **PX4 SITL**

### What Went Wrong
By default, when you run `System()`, MAVSDK-Python tries to connect to or start a `mavsdk_server` on **gRPC port 50051**.

Because `SwarmManager` was initializing systems like this:
```python
self.drones[drone_id] = {
    "system": System(),
    "port": port
}
```
Both `Drone 1` and `Drone 2` initialized a `System` object without specifying a gRPC port. 
- Drone 1 started `mavsdk_server` on port 50051 and connected it to UDP `14540`.
- Drone 2 **re-used** the already running `mavsdk_server` on port 50051 and told it to *also* connect to UDP `14541`.

Because both Python `System` objects were talking to the exact same `mavsdk_server` process, the server simply merged the connections or routed commands to the first vehicle it discovered. When you sent a `LAND` command to Drone 2, it was effectively ignored or sent to the wrong vehicle by the confused `mavsdk_server`.

## The Solution

To control multiple vehicles from a single Python script, **each drone must have its own dedicated `mavsdk_server` process**. We enforce this by assigning a unique gRPC port to each `System` object.

### The Fix
In `swarm_manager.py`, the initialization was updated to assign a unique port based on the `drone_id`:

```python
def add_drone(self, drone_id, port):
    # Calculate a unique gRPC port for each drone
    # e.g., Drone 1 -> 50051, Drone 2 -> 50052
    grpc_port = 50050 + drone_id
    
    self.drones[drone_id] = {
        "system": System(port=grpc_port),
        "port": port
    }
```

Now, the architecture looks like this:
- **Drone 1**: Python `System(port=50051)` -> `mavsdk_server` (Port 50051) -> PX4 (UDP 14540)
- **Drone 2**: Python `System(port=50052)` -> `mavsdk_server` (Port 50052) -> PX4 (UDP 14541)

Because they are completely isolated, sending a command to Drone 2 will explicitly route through its own `mavsdk_server` and only affect Drone 2 in Gazebo.

## Next Steps & Verification

With this fix applied, you can now reliably verify multi-drone interaction:
1. **Independent Telemetry**: You can stream `await drone.telemetry.position()` for both drones and see distinct coordinates.
2. **Independent Control**: Commands like `/drone_1/mission_command LAND` and `/drone_2/mission_command LAND` will now correctly trigger actions on their respective vehicles.
3. **Formation Flying**: You can confidently send Offboard `GOTO` commands to multiple drones simultaneously.

*Note: As your swarm grows, ensure that your MAV_SYS_ID (1, 2, 3...) in PX4 matches the logical grouping, though the separated `mavsdk_server` instances ensure that routing is secure regardless of the sysid.*
