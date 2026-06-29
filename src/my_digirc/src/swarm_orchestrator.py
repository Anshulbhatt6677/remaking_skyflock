#!/usr/bin/env python3

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
from my_digirc.srv import Command


# Number of drones — configurable via environment variable
NUM_DRONES = int(os.environ.get("NUM_DRONES", "3"))

# Spawn spacing along X axis (must match start_swarm.sh offsets)
SPAWN_SPACING_X = float(os.environ.get("SPAWN_SPACING_X", "5.0"))


class SwarmOrchestrator(Node):
    """
    High-level swarm brain.

    Subscribes to /swarm/command for movement and formation commands,
    maintains swarm center state, calculates per-drone positions, and
    dispatches GOTO commands to each drone's mission_command service.
    """

    def __init__(self):
        super().__init__("swarm_orchestrator")

        self.num_drones = NUM_DRONES

        # Service clients for all drones
        self.drone_clients = {}

        # Generate spawn offsets dynamically:
        # Drone 1 at origin, Drone 2 at (SPACING, 0), Drone 3 at (2*SPACING, 0), etc.
        self.spawn_offsets = {
            i: ((i - 1) * SPAWN_SPACING_X, 0.0)
            for i in range(1, self.num_drones + 1)
        }

        # Swarm center and active shape state
        self.center_x = 10.0
        self.center_y = 0.0
        self.center_z = -8.0
        self.center_yaw = 0.0
        self.current_formation = None
        self.mission_active = False

        # Revolution state (orbiting drones around center)
        self.revolution_active = False
        self.revolution_speed = 10.0      # deg/s, sign = direction
        self.revolution_angle = 0.0       # current cumulative angle (degrees)
        self.orbit_radius = 15.0          # metres
        self._revolution_timer = None

        # Rotation state (spinning drones about own yaw axis)
        self.rotation_active = False
        self.rotation_speed = 30.0        # deg/s, sign = direction
        self.rotation_angle = 0.0         # current cumulative yaw (degrees)
        self._rotation_timer = None

        # Tick interval for orbit timers (seconds)
        self._tick_dt = 0.1               # 10 Hz

        for i in range(1, self.num_drones + 1):
            client = self.create_client(Command, f"/drone_{i}/mission_command")
            self.drone_clients[i] = client
            if not client.service_is_ready():
                self.get_logger().info(
                    f"Waiting for /drone_{i}/mission_command service..."
                )

        # QoS: Best-effort for yaw (must match subscriber's QoS)
        yaw_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # QoS: Reliable for commands (must not be dropped)
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Yaw publishers
        self.yaw_pubs = {
            i: self.create_publisher(Float32, f"/drone_{i}/yaw_command", yaw_qos)
            for i in range(1, self.num_drones + 1)
        }

        # Subscribe to the central swarm command topic
        self.create_subscription(
            String, "/swarm/command", self.command_callback, command_qos
        )

        self.get_logger().info(
            f"SwarmOrchestrator ready — {self.num_drones} drones, "
            f"spawn offsets: {self.spawn_offsets}"
        )

    # ------------------------------------------------------------------
    # Command Sending
    # ------------------------------------------------------------------
    def send_command(self, drone_id, command_str, x=0.0, y=0.0, z=0.0, yaw=0.0, quiet=False):
        """Send a command to a single drone, adjusting for spawn offsets on GOTO."""
        if not self.drone_clients[drone_id].service_is_ready():
            if not quiet:
                self.get_logger().warning(
                    f"Service for drone {drone_id} is not ready!"
                )
            return

        req = Command.Request()
        req.command = command_str

        # Subtract spawn offset so all drones operate in a shared global frame
        if command_str == "GOTO":
            offset_x, offset_y = self.spawn_offsets.get(drone_id, (0.0, 0.0))
            req.x = float(x - offset_x)
            req.y = float(y - offset_y)
        else:
            req.x = float(x)
            req.y = float(y)

        req.z = float(z)
        req.yaw = float(yaw)

        self.drone_clients[drone_id].call_async(req)
        if not quiet:
            self.get_logger().info(f"Sent {command_str} to Drone {drone_id}")

    # ------------------------------------------------------------------
    # Command Dispatch
    # ------------------------------------------------------------------
    def command_callback(self, msg):
        """Route incoming swarm commands to the appropriate handler."""
        cmd = msg.data.upper()
        self.get_logger().info(f"Received swarm command: {cmd}")

        # Abort active mission on any new command (except SQUARE itself)
        if cmd != "SQUARE":
            if self.mission_active:
                self.get_logger().info(
                    f"Manual command '{cmd}' received. Canceling active mission."
                )
            self.mission_active = False

        if cmd == "TAKEOFF":
            self.takeoff_all()
        elif cmd == "OFFBOARD":
            self.start_offboard_all()
        elif cmd == "V":
            self.stop_revolution()
            self.current_formation = "V"
            self.update_formation()
        elif cmd == "LINE":
            self.stop_revolution()
            self.current_formation = "LINE"
            self.update_formation()
        elif cmd == "HOVER":
            self.stop_revolution()
            self.stop_rotation()
            self.hover_all()
        elif cmd == "LAND":
            self.stop_revolution()
            self.stop_rotation()
            self.land_all()
        elif cmd == "FORWARD":
            self.center_x += 1.0
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "BACKWARD":
            self.center_x -= 1.0
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "LEFT":
            self.center_y -= 1.0
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "RIGHT":
            self.center_y += 1.0
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "UP":
            self.center_z -= 1.0  # Up in NED is negative
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "DOWN":
            self.center_z += 1.0  # Down in NED is positive
            if not self.revolution_active:
                self.update_formation()
        elif cmd == "ROTATE_LEFT":
            self.center_yaw -= 15.0
            self.update_yaw_all()
        elif cmd == "ROTATE_RIGHT":
            self.center_yaw += 15.0
            self.update_yaw_all()
        elif cmd == "SQUARE":
            self.trigger_square_mission()
        elif cmd == "REVOLVE":
            self.toggle_revolution()
        elif cmd == "ROTATE":
            self.toggle_rotation()
        elif cmd == "REVERSE_REVOLUTION":
            self.revolution_speed = -self.revolution_speed
            self.get_logger().info(
                f"Revolution direction reversed (speed={self.revolution_speed}°/s)"
            )
        elif cmd == "REVERSE_ROTATION":
            self.rotation_speed = -self.rotation_speed
            self.get_logger().info(
                f"Rotation direction reversed (speed={self.rotation_speed}°/s)"
            )
        else:
            self.get_logger().warning(f"Unknown command: {cmd}")

    # ------------------------------------------------------------------
    # Formations
    # ------------------------------------------------------------------
    def update_formation(self):
        """Recalculate and dispatch the current formation."""
        if self.current_formation == "V":
            self.form_v(self.center_x, self.center_y, self.center_z)
        elif self.current_formation == "LINE":
            self.form_line(self.center_x, self.center_y, self.center_z)
        else:
            self.get_logger().warning(
                "Cannot update formation: no shape has been set yet."
            )

    def form_v(self, center_x, center_y, center_z, spacing=5.0):
        """
        Arrange drones in a V-formation.
        Drone 1 is the leader at the center; wings extend behind and outward.
        """
        self.get_logger().info(
            f"Forming V at x={center_x}, y={center_y}, z={center_z}"
        )

        # Leader
        self.send_command(1, "GOTO", center_x, center_y, center_z, self.center_yaw)

        # Wings — dynamically sized based on NUM_DRONES
        wing_pair = 1
        for i in range(2, self.num_drones + 1, 2):
            # Left wing
            self.send_command(
                i, "GOTO",
                center_x - spacing * wing_pair,
                center_y - spacing * wing_pair,
                center_z, self.center_yaw,
            )
            # Right wing (if there's a drone for it)
            if i + 1 <= self.num_drones:
                self.send_command(
                    i + 1, "GOTO",
                    center_x - spacing * wing_pair,
                    center_y + spacing * wing_pair,
                    center_z, self.center_yaw,
                )
            wing_pair += 1

    def form_line(self, center_x, center_y, center_z, spacing=5.0):
        """Arrange drones in a line along the Y axis."""
        self.get_logger().info(
            f"Forming Line at x={center_x}, y={center_y}, z={center_z}"
        )

        # Center drone 1, then alternate left/right for the rest
        self.send_command(1, "GOTO", center_x, center_y, center_z, self.center_yaw)

        offset_index = 1
        for i in range(2, self.num_drones + 1, 2):
            # Left
            self.send_command(
                i, "GOTO",
                center_x,
                center_y - spacing * offset_index,
                center_z, self.center_yaw,
            )
            # Right
            if i + 1 <= self.num_drones:
                self.send_command(
                    i + 1, "GOTO",
                    center_x,
                    center_y + spacing * offset_index,
                    center_z, self.center_yaw,
                )
            offset_index += 1

    # ------------------------------------------------------------------
    # Revolution (orbiting drones around center)
    # ------------------------------------------------------------------
    def toggle_revolution(self):
        """Toggle revolution mode on/off."""
        if self.revolution_active:
            self.stop_revolution()
        else:
            self.start_revolution()

    def start_revolution(self):
        """Start revolving drones around the swarm center."""
        if self.revolution_active:
            return
        self.revolution_active = True
        self.revolution_angle = 0.0
        self.get_logger().info(
            f"Revolution started (radius={self.orbit_radius}m, "
            f"speed={self.revolution_speed}°/s)"
        )
        # If rotation is also active, its timer will be paused;
        # the revolution tick handles both.
        if self.rotation_active and self._rotation_timer is not None:
            self._rotation_timer.cancel()
            self._rotation_timer = None

        self._revolution_timer = self.create_timer(
            self._tick_dt, self._revolution_tick
        )

    def stop_revolution(self):
        """Stop revolution. Drones hold their last positions."""
        if not self.revolution_active:
            return
        self.revolution_active = False
        if self._revolution_timer is not None:
            self._revolution_timer.cancel()
            self._revolution_timer = None
        self.get_logger().info("Revolution stopped.")

        # If rotation is still active, restart its own timer
        if self.rotation_active and self._rotation_timer is None:
            self._rotation_timer = self.create_timer(
                self._tick_dt, self._rotation_tick
            )

    def _revolution_tick(self):
        """
        Timer callback at ~10 Hz.
        Advances the orbit angle and repositions all drones on the circle.
        If rotation is also active, advances yaw too.
        """
        self.revolution_angle += self.revolution_speed * self._tick_dt

        # Advance rotation yaw if rotation mode is also active
        if self.rotation_active:
            self.rotation_angle += self.rotation_speed * self._tick_dt

        angle_step = 360.0 / self.num_drones

        for i in range(1, self.num_drones + 1):
            # Calculate an offset angle that matches the V/Line formation structure:
            # Drone 1 (Leader): 0 offset
            # Drone 2 (Left): -angle_step
            # Drone 3 (Right): +angle_step
            # Drone 4 (Left): -2 * angle_step, etc.
            if i == 1:
                offset_multiplier = 0
            elif i % 2 == 0:
                # Evens go Left (negative angle)
                offset_multiplier = -(i // 2)
            else:
                # Odds go Right (positive angle)
                offset_multiplier = (i // 2)

            drone_offset_angle = offset_multiplier * angle_step
            drone_angle_deg = self.revolution_angle + drone_offset_angle
            drone_angle_rad = math.radians(drone_angle_deg)

            x = self.center_x + self.orbit_radius * math.cos(drone_angle_rad)
            y = self.center_y + self.orbit_radius * math.sin(drone_angle_rad)

            # Yaw: use rotation_angle if rotation is active, else face the direction of flight (tangent)
            if self.rotation_active:
                yaw = self.rotation_angle
            else:
                tangent_yaw = drone_angle_deg + (90 if self.revolution_speed > 0 else -90)
                yaw = (tangent_yaw + 180) % 360 - 180

            self.send_command(i, "GOTO", x, y, self.center_z, yaw, quiet=True)

    # ------------------------------------------------------------------
    # Rotation (spinning drones about their own yaw axis)
    # ------------------------------------------------------------------
    def toggle_rotation(self):
        """Toggle rotation mode on/off."""
        if self.rotation_active:
            self.stop_rotation()
        else:
            self.start_rotation()

    def start_rotation(self):
        """Start spinning all drones about their own yaw axis."""
        if self.rotation_active:
            return
        self.rotation_active = True
        self.rotation_angle = self.center_yaw  # Start from current heading
        self.get_logger().info(
            f"Rotation started (speed={self.rotation_speed}°/s)"
        )
        # If revolution is active, it handles yaw too — no separate timer needed.
        if not self.revolution_active:
            self._rotation_timer = self.create_timer(
                self._tick_dt, self._rotation_tick
            )

    def stop_rotation(self):
        """Stop rotation. Yaw freezes at current angle."""
        if not self.rotation_active:
            return
        self.rotation_active = False
        if self._rotation_timer is not None:
            self._rotation_timer.cancel()
            self._rotation_timer = None
        # Update center_yaw to the final rotation angle so future
        # formations / manual yaw adjustments start from here.
        self.center_yaw = self.rotation_angle
        self.get_logger().info(
            f"Rotation stopped (yaw frozen at {self.center_yaw:.1f}°)"
        )

    def _rotation_tick(self):
        """
        Timer callback at ~10 Hz.
        Advances yaw and publishes to all drones.
        Only runs when revolution is NOT active (otherwise revolution_tick
        handles yaw to avoid desync).
        """
        self.rotation_angle += self.rotation_speed * self._tick_dt
        msg = Float32()
        msg.data = self.rotation_angle
        for i in range(1, self.num_drones + 1):
            self.yaw_pubs[i].publish(msg)

    # ------------------------------------------------------------------
    # Bulk Operations
    # ------------------------------------------------------------------
    def update_yaw_all(self):
        """Broadcast the current center yaw to all drones."""
        msg = Float32()
        msg.data = self.center_yaw
        for i in range(1, self.num_drones + 1):
            self.yaw_pubs[i].publish(msg)
        self.get_logger().info(
            f"Updated swarm yaw to {self.center_yaw} degrees"
        )

    def takeoff_all(self):
        for i in range(1, self.num_drones + 1):
            self.send_command(i, "TAKEOFF")

    def start_offboard_all(self):
        for i in range(1, self.num_drones + 1):
            self.send_command(i, "START_OFFBOARD")

    def hover_all(self):
        for i in range(1, self.num_drones + 1):
            self.send_command(i, "HOVER")

    def land_all(self):
        for i in range(1, self.num_drones + 1):
            self.send_command(i, "LAND")

    # ------------------------------------------------------------------
    # Missions
    # ------------------------------------------------------------------
    def trigger_square_mission(self):
        """Start a 10x10 square flight pattern in a background thread."""
        if self.mission_active:
            self.get_logger().warning("Mission already active!")
            return
        threading.Thread(target=self.execute_square_mission, daemon=True).start()

    def execute_square_mission(self):
        """Fly all drones through a 10x10 square pattern. Abortable via mission_active flag."""
        self.mission_active = True
        self.get_logger().info("Starting square mission. 10x10 pattern.")
        base_x = self.center_x
        base_y = self.center_y
        base_z = self.center_z

        waypoints = [
            (base_x + 10.0, base_y, base_z),
            (base_x + 10.0, base_y + 10.0, base_z),
            (base_x, base_y + 10.0, base_z),
            (base_x, base_y, base_z),
        ]

        for i, (wp_x, wp_y, wp_z) in enumerate(waypoints):
            if not self.mission_active:
                self.get_logger().info("Square mission aborted.")
                return

            self.get_logger().info(
                f"Mission Waypoint {i + 1}/4: x={wp_x}, y={wp_y}"
            )
            self.center_x = wp_x
            self.center_y = wp_y
            self.center_z = wp_z
            self.update_formation()

            # Wait 8 seconds for drones to travel, checking for aborts frequently
            for _ in range(80):
                if not self.mission_active:
                    self.get_logger().info("Square mission aborted.")
                    return
                time.sleep(0.1)

        self.mission_active = False
        self.get_logger().info("Square mission complete.")


def main(args=None):
    rclpy.init(args=args)
    node = SwarmOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
