"""
Connection management for MAVSDK drones.

Handles connecting to individual drones via UDP and starting their
background telemetry/battery tasks once connected.
"""

import asyncio
from mavsdk import System


async def connect_drone(drones, drone_id, logger, on_connected_callbacks=None):
    """
    Connect to a single drone and run initial health check.

    Args:
        drones: dict of drone state dicts keyed by drone_id
        drone_id: integer drone identifier
        logger: logging object with .info/.warn/.error methods
        on_connected_callbacks: optional list of async callables(drone_id) to run after connection
    """
    drone = drones[drone_id]["system"]
    port = drones[drone_id]["port"]

    logger.info(f"Drone {drone_id}: connecting on UDP port {port}...")

    await drone.connect(system_address=f"udpin://0.0.0.0:{port}")

    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info(f"Drone {drone_id}: connected")
            break

    # Log initial health status
    async for health in drone.telemetry.health():
        logger.info(
            f"Drone {drone_id}: health — "
            f"global_pos={health.is_global_position_ok}, "
            f"home_pos={health.is_home_position_ok}"
        )
        break

    # Run any post-connection callbacks (e.g. start telemetry/battery loops)
    if on_connected_callbacks:
        for cb in on_connected_callbacks:
            asyncio.create_task(cb(drone_id))


async def connect_all_drones(drones, logger, on_connected_callbacks=None):
    """Connect to all registered drones concurrently."""
    tasks = [
        connect_drone(drones, drone_id, logger, on_connected_callbacks)
        for drone_id in drones
    ]
    await asyncio.gather(*tasks)
