"""
SwarmManager — composed from focused sub-modules.

This is the main entry point for drone management. It delegates to:
  - connection.py  — connecting to drones
  - commands.py    — flight commands (arm, takeoff, land, goto, etc.)
  - telemetry.py   — position streaming and battery monitoring
  - setpoint.py    — continuous offboard setpoint streaming
"""

import logging
from mavsdk import System

from manager.connection import connect_all_drones
from manager.commands import (
    VALID_COMMANDS,
    arm_drone,
    takeoff_drone,
    land_drone,
    goto_position,
    start_offboard,
    hover_drone,
    check_health,
)
from manager.telemetry import telemetry_loop, battery_monitor_loop, health_monitor_loop, home_monitor_loop
from manager.setpoint import setpoint_loop, update_yaw


class SwarmManager:
    """
    Manages multiple drones via MAVSDK.

    Handles connection, commands, telemetry streaming, battery failsafe,
    and continuous setpoint streaming. Accepts an optional logger (e.g. a
    ROS 2 logger) for structured output; falls back to Python's logging
    module for standalone use.

    Public API is unchanged from the monolithic version — existing code
    (swarm_controller.py) works without modification.
    """

    def __init__(self, logger=None):
        self.drones = {}
        self.telemetry_callbacks = []
        self.battery_callbacks = []
        self.health_callbacks = []

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
    # Drone Registration
    # ------------------------------------------------------------------
    def register_telemetry_callback(self, callback):
        """Register a callable(drone_id, x, y, z) to receive telemetry updates."""
        self.telemetry_callbacks.append(callback)

    def register_battery_callback(self, callback):
        """Register a callable(drone_id, battery_percent) to receive battery updates."""
        self.battery_callbacks.append(callback)

    def register_health_callback(self, callback):
        """Register a callable(drone_id, global_ok, home_ok) to receive health updates."""
        self.health_callbacks.append(callback)

    def add_drone(self, drone_id, port):
        """Register a new drone with a UDP port and unique gRPC port."""
        grpc_port = 50050 + drone_id
        self.drones[drone_id] = {
            "system": System(port=grpc_port),
            "port": port,
            "target_position": None,
            "setpoint_task": None,
        }
        self._logger.info(f"Drone {drone_id}: registered (UDP={port}, gRPC={grpc_port})")

    # ------------------------------------------------------------------
    # Connection (delegates to connection.py)
    # ------------------------------------------------------------------
    async def connect_all_drones(self):
        """Connect to all registered drones concurrently."""
        # After connection, start telemetry + battery loops
        from manager.telemetry import health_monitor_loop
        async def _start_telemetry(drone_id):
            await telemetry_loop(self.drones, drone_id, self.telemetry_callbacks, self._logger)

        async def _start_battery_monitor(drone_id):
            await battery_monitor_loop(self.drones, drone_id, self._logger, self.battery_callbacks)

        async def _start_health_monitor(drone_id):
            await health_monitor_loop(self.drones, drone_id, self._logger, self.health_callbacks)

        async def _start_home_monitor(drone_id):
            await home_monitor_loop(self.drones, drone_id, self._logger)

        await connect_all_drones(
            self.drones,
            self._logger,
            on_connected_callbacks=[_start_telemetry, _start_battery_monitor, _start_health_monitor, _start_home_monitor],
        )

    # ------------------------------------------------------------------
    # Setpoint (delegates to setpoint.py)
    # ------------------------------------------------------------------
    def update_yaw(self, drone_id, yaw_deg):
        """Update the yaw component of the current target position."""
        update_yaw(self.drones, drone_id, yaw_deg, self._logger)

    # ------------------------------------------------------------------
    # Command Dispatch (delegates to commands.py)
    # ------------------------------------------------------------------
    async def execute_command(self, drone_id, command, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """
        Dispatch a command string to the appropriate handler.
        Validates the command before execution.
        """
        command = command.upper().strip()

        if command not in VALID_COMMANDS:
            self._logger.error(
                f"Drone {drone_id}: rejected unknown command '{command}'. "
                f"Valid commands: {', '.join(sorted(VALID_COMMANDS))}"
            )
            return

        if drone_id not in self.drones:
            self._logger.error(f"Drone {drone_id}: not registered")
            return

        if command == "ARM":
            await arm_drone(self.drones, drone_id, self._logger)
        elif command == "TAKEOFF":
            await takeoff_drone(self.drones, drone_id, self._logger)
        elif command == "LAND":
            await land_drone(self.drones, drone_id, self._logger)
        elif command == "GOTO":
            await goto_position(self.drones, drone_id, x, y, z, yaw, self._logger)
        elif command == "START_OFFBOARD":
            # Pass the setpoint loop function so offboard can start the background task
            await start_offboard(
                self.drones, drone_id, self._logger,
                setpoint_loop_fn=lambda did: setpoint_loop(self.drones, did, self._logger),
            )
        elif command == "HOVER":
            await hover_drone(self.drones, drone_id, self._logger)
