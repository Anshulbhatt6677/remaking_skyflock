#!/usr/bin/env python3

import asyncio
import logging
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw


# Valid commands accepted by execute_command()
VALID_COMMANDS = {"ARM", "TAKEOFF", "LAND", "GOTO", "START_OFFBOARD", "HOVER"}

# Safety limits
MAX_POSITION_M = 500.0    # Max absolute coordinate value (metres)
MAX_ALTITUDE_M = 100.0    # Max altitude (NED down, so this is the min z value)
BATTERY_CRITICAL_THRESHOLD = 0.20  # 20%


class SwarmManager:
    """
    Manages multiple drones via MAVSDK.

    Handles connection, commands, telemetry streaming, battery failsafe,
    and continuous setpoint streaming. Accepts an optional logger (e.g. a
    ROS 2 logger) for structured output; falls back to Python's logging
    module for standalone use.
    """

    def __init__(self, logger=None):
        self.drones = {}
        self.telemetry_callbacks = []
        self.ref_alt = None

        # Accept a ROS logger or fall back to Python's stdlib logger
        if logger is not None:
            self._logger = logger
        else:
            self._logger = logging.getLogger("SwarmManager")
            if not self._logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter(
                    "[%(name)s] %(levelname)s: %(message)s"
                ))
                self._logger.addHandler(handler)
                self._logger.setLevel(logging.INFO)

    # ------------------------------------------------------------------
    # Logging helpers — wrap the logger so it works with both ROS loggers
    # (which have .info/.warn/.error) and Python loggers (same API).
    # ------------------------------------------------------------------
    def _log_info(self, msg):
        self._logger.info(msg)

    def _log_warn(self, msg):
        # ROS loggers use .warning(), Python loggers also accept .warning()
        if hasattr(self._logger, 'warn'):
            self._logger.warn(msg)
        else:
            self._logger.warning(msg)

    def _log_error(self, msg):
        self._logger.error(msg)

    # ------------------------------------------------------------------
    # Drone Registration
    # ------------------------------------------------------------------
    def register_telemetry_callback(self, callback):
        """Register a callable(drone_id, x, y, z) to receive telemetry updates."""
        self.telemetry_callbacks.append(callback)

    def add_drone(self, drone_id, port):
        """Register a new drone with a UDP port and unique gRPC port."""
        grpc_port = 50050 + drone_id

        self.drones[drone_id] = {
            "system": System(port=grpc_port),
            "port": port,
            "target_position": None,
            "setpoint_task": None,
        }
        self._log_info(f"Drone {drone_id}: registered (UDP={port}, gRPC={grpc_port})")

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------
    async def connect_drone(self, drone_id):
        """Connect to a single drone and start its background tasks."""
        drone = self.drones[drone_id]["system"]
        port = self.drones[drone_id]["port"]

        self._log_info(f"Drone {drone_id}: connecting on UDP port {port}...")

        await drone.connect(system_address=f"udpin://0.0.0.0:{port}")

        async for state in drone.core.connection_state():
            if state.is_connected:
                self._log_info(f"Drone {drone_id}: connected")
                break

        # Log initial health status
        async for health in drone.telemetry.health():
            self._log_info(
                f"Drone {drone_id}: health — "
                f"global_pos={health.is_global_position_ok}, "
                f"home_pos={health.is_home_position_ok}"
            )
            break

        # Wait for home position to get absolute altitude
        async for home in drone.telemetry.home():
            self._log_info(f"Drone {drone_id}: home altitude is {home.absolute_altitude_m}m")
            self.drones[drone_id]["home_alt"] = home.absolute_altitude_m
            # If this is the first drone to connect, use its altitude as reference
            if self.ref_alt is None:
                self.ref_alt = home.absolute_altitude_m
                self._log_info(f"Reference altitude set to {self.ref_alt}m")
            break

        # Start background tasks
        asyncio.create_task(self._telemetry_loop(drone_id))
        asyncio.create_task(self._battery_monitor_loop(drone_id))

    async def connect_all_drones(self):
        """Connect to all registered drones concurrently."""
        tasks = [self.connect_drone(drone_id) for drone_id in self.drones]
        await asyncio.gather(*tasks)

    # ------------------------------------------------------------------
    # Telemetry Streaming
    # ------------------------------------------------------------------
    async def _telemetry_loop(self, drone_id):
        """Stream NED position from PX4 and forward to registered callbacks."""
        drone = self.drones[drone_id]["system"]
        error_count = 0

        async for pos_vel in drone.telemetry.position_velocity_ned():
            pos = pos_vel.position
            for cb in self.telemetry_callbacks:
                try:
                    cb(drone_id, pos.north_m, pos.east_m, pos.down_m)
                except Exception as e:
                    error_count += 1
                    # Log every 100th error to avoid spam
                    if error_count % 100 == 1:
                        self._log_warn(
                            f"Drone {drone_id}: telemetry callback error "
                            f"(count={error_count}): {e}"
                        )

    # ------------------------------------------------------------------
    # Battery Failsafe
    # ------------------------------------------------------------------
    async def _battery_monitor_loop(self, drone_id):
        """Monitor battery and trigger RTL if critically low."""
        drone = self.drones[drone_id]["system"]

        async for battery in drone.telemetry.battery():
            if battery.remaining_percent < BATTERY_CRITICAL_THRESHOLD:
                self._log_error(
                    f"CRITICAL: Drone {drone_id} battery low "
                    f"({battery.remaining_percent * 100:.1f}%). Triggering RTL!"
                )

                # Cancel the streaming task so it doesn't fight RTL mode
                task = self.drones[drone_id].get("setpoint_task")
                if task is not None:
                    task.cancel()
                    self.drones[drone_id]["setpoint_task"] = None

                try:
                    await drone.action.return_to_launch()
                    self._log_info(f"Drone {drone_id}: RTL triggered successfully")
                except Exception as e:
                    self._log_error(f"Drone {drone_id}: RTL failed — {e}")

                break  # Exit the monitor loop once RTL is triggered

    # ------------------------------------------------------------------
    # Setpoint Streaming
    # ------------------------------------------------------------------
    async def _setpoint_loop(self, drone_id):
        """
        Continuously stream the current target_position to PX4 at 10 Hz.
        If target_position is None, streams zero-velocity (hover).
        """
        drone = self.drones[drone_id]["system"]

        while True:
            try:
                target = self.drones[drone_id].get("target_position")
                if target is not None:
                    await drone.offboard.set_position_ned(target)
                else:
                    await drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                    )
            except Exception as e:
                self._log_warn(f"Drone {drone_id}: setpoint stream error — {e}")
            await asyncio.sleep(0.1)

    def update_yaw(self, drone_id, yaw_deg):
        """Update the yaw component of the current target position."""
        target = self.drones[drone_id].get("target_position")
        if target is not None:
            self.drones[drone_id]["target_position"] = PositionNedYaw(
                target.north_m,
                target.east_m,
                target.down_m,
                float(yaw_deg),
            )
            self._log_info(f"Drone {drone_id}: yaw updated to {yaw_deg}°")
        else:
            self._log_warn(
                f"Drone {drone_id}: cannot update yaw — no active target position"
            )

    # ------------------------------------------------------------------
    # Input Validation
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp(value, min_val, max_val):
        """Clamp a value to [min_val, max_val]."""
        return max(min_val, min(max_val, value))

    def _validate_position(self, x, y, z):
        """Clamp position values to safe ranges and return (x, y, z)."""
        x = self._clamp(float(x), -MAX_POSITION_M, MAX_POSITION_M)
        y = self._clamp(float(y), -MAX_POSITION_M, MAX_POSITION_M)
        # NED: negative z = up, so clamp to [-MAX_ALTITUDE, some_positive_value]
        z = self._clamp(float(z), -MAX_ALTITUDE_M, 10.0)
        return x, y, z

    # ------------------------------------------------------------------
    # Flight Commands
    # ------------------------------------------------------------------
    async def arm_drone(self, drone_id):
        """Arm a single drone."""
        drone = self.drones[drone_id]["system"]
        try:
            await drone.action.arm()
            self._log_info(f"Drone {drone_id}: armed")
        except Exception as e:
            self._log_error(f"Drone {drone_id}: ARM ERROR — {e}")

    async def takeoff_drone(self, drone_id):
        """Run the full takeoff sequence: health check → hold → arm → takeoff."""
        drone = self.drones[drone_id]["system"]
        self._log_info(f"Drone {drone_id}: starting takeoff sequence")

        # Wait for GPS/health using the background monitor's cached state
        for attempt in range(30):
            global_ok = self.drones[drone_id].get("is_global_position_ok", False)
            home_ok = self.drones[drone_id].get("is_home_position_ok", False)
            
            self._log_info(f"Drone {drone_id}: health — global={global_ok}, home={home_ok}")
            if global_ok and home_ok:
                break
            await asyncio.sleep(1.0)
        else:
            self._log_error(f"Drone {drone_id}: Health check timed out! Cannot takeoff.")
            return

        # Switch to hold to clear any stale land state
        try:
            self._log_info(f"Drone {drone_id}: switching to HOLD mode")
            await drone.action.hold()
            await asyncio.sleep(1)
        except Exception as e:
            self._log_warn(f"Drone {drone_id}: HOLD warning — {e}")

        # Set Takeoff Altitude BEFORE Arming (PX4 rejects param changes while armed)
        try:
            offset = 0.0
            if self.ref_alt is not None and "home_alt" in self.drones[drone_id]:
                offset = self.ref_alt - self.drones[drone_id]["home_alt"]
            takeoff_alt = 8.0 + offset
            
            self._log_info(f"Drone {drone_id}: setting takeoff altitude to {takeoff_alt:.2f}m (offset: {offset:.2f}m)")
            await drone.action.set_takeoff_altitude(takeoff_alt)
        except Exception as e:
            self._log_warn(f"Drone {drone_id}: could not set takeoff altitude — {e}")

        # Arm with retries
        for attempt in range(3):
            try:
                self._log_info(f"Drone {drone_id}: arming (attempt {attempt+1}/3)")
                await drone.action.arm()
                await asyncio.sleep(1.0)
                break
            except Exception as e:
                self._log_warn(f"Drone {drone_id}: ARM warning — {e}")
                await asyncio.sleep(1.0)

        # Takeoff with retries
        for attempt in range(3):
            try:
                self._log_info(f"Drone {drone_id}: sending takeoff command (attempt {attempt+1}/3)")
                await drone.action.takeoff()
                self._log_info(f"Drone {drone_id}: takeoff command sent")
                break
            except Exception as e:
                self._log_error(f"Drone {drone_id}: TAKEOFF ERROR — {e}")
                await asyncio.sleep(1.5)

    async def land_drone(self, drone_id):
        """Cancel setpoint streaming and land."""
        # Cancel the setpoint task so it doesn't fight the landing
        task = self.drones[drone_id].get("setpoint_task")
        if task is not None:
            task.cancel()
            self.drones[drone_id]["setpoint_task"] = None

        drone = self.drones[drone_id]["system"]
        self._log_info(f"Drone {drone_id}: landing")
        await drone.action.land()
        self._log_info(f"Drone {drone_id}: land command sent")

    async def goto_position(self, drone_id, x, y, z, yaw):
        """Update the target position for the setpoint streaming loop."""
        x, y, z = self._validate_position(x, y, z)
        
        offset = 0.0
        if self.ref_alt is not None and "home_alt" in self.drones[drone_id]:
            offset = self.ref_alt - self.drones[drone_id]["home_alt"]
            
        z_corrected = z - offset
        self.drones[drone_id]["target_position"] = PositionNedYaw(x, y, z_corrected, yaw)

    async def start_offboard(self, drone_id):
        """Prime setpoints and start offboard mode + background streaming."""
        drone = self.drones[drone_id]["system"]
        self._log_info(f"Drone {drone_id}: starting offboard mode")

        # Prime with current target or hover
        target = self.drones[drone_id].get("target_position")
        if target is not None:
            await drone.offboard.set_position_ned(target)
        else:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )

        await drone.offboard.start()
        self._log_info(f"Drone {drone_id}: offboard started")

        # Start the background setpoint streaming task
        if self.drones[drone_id].get("setpoint_task") is None:
            self.drones[drone_id]["setpoint_task"] = asyncio.create_task(
                self._setpoint_loop(drone_id)
            )

    async def hover_drone(self, drone_id):
        """Force hover by clearing the target position (setpoint loop sends zero velocity)."""
        self._log_info(f"Drone {drone_id}: hovering in place")
        self.drones[drone_id]["target_position"] = None

    async def check_health(self, drone_id):
        """Print the current health status of a drone."""
        drone = self.drones[drone_id]["system"]
        async for health in drone.telemetry.health():
            self._log_info(
                f"Drone {drone_id}: "
                f"global_pos={health.is_global_position_ok}, "
                f"home_pos={health.is_home_position_ok}"
            )
            break

    # ------------------------------------------------------------------
    # Command Dispatch
    # ------------------------------------------------------------------
    async def execute_command(self, drone_id, command, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """
        Dispatch a command string to the appropriate handler.
        Validates the command before execution.
        """
        command = command.upper().strip()

        if command not in VALID_COMMANDS:
            self._log_error(
                f"Drone {drone_id}: rejected unknown command '{command}'. "
                f"Valid commands: {', '.join(sorted(VALID_COMMANDS))}"
            )
            return

        if drone_id not in self.drones:
            self._log_error(f"Drone {drone_id}: not registered")
            return

        if command == "ARM":
            await self.arm_drone(drone_id)
        elif command == "TAKEOFF":
            await self.takeoff_drone(drone_id)
        elif command == "LAND":
            await self.land_drone(drone_id)
        elif command == "GOTO":
            await self.goto_position(drone_id, x, y, z, yaw)
        elif command == "START_OFFBOARD":
            await self.start_offboard(drone_id)
        elif command == "HOVER":
            await self.hover_drone(drone_id)
