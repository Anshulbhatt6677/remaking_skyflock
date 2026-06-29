"""
Setpoint streaming for MAVSDK offboard control.

Handles the 10 Hz background loop that continuously sends position
or velocity commands to keep PX4 in OFFBOARD mode.
"""

import asyncio
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw


async def setpoint_loop(drones, drone_id, logger):
    """
    Continuously stream the current target_position to PX4 at 10 Hz.
    If target_position is None, streams zero-velocity (hover).
    Runs indefinitely as a background task.
    """
    drone = drones[drone_id]["system"]

    while True:
        try:
            target = drones[drone_id].get("target_position")
            if target is not None:
                await drone.offboard.set_position_ned(target)
            else:
                await drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
        except Exception as e:
            if hasattr(logger, 'warn'):
                logger.warn(f"Drone {drone_id}: setpoint stream error — {e}")
            else:
                logger.warning(f"Drone {drone_id}: setpoint stream error — {e}")
        await asyncio.sleep(0.1)


def update_yaw(drones, drone_id, yaw_deg, logger):
    """Update the yaw component of the current target position."""
    target = drones[drone_id].get("target_position")
    if target is not None:
        drones[drone_id]["target_position"] = PositionNedYaw(
            target.north_m,
            target.east_m,
            target.down_m,
            float(yaw_deg),
        )
        logger.info(f"Drone {drone_id}: yaw updated to {yaw_deg}°")
    else:
        if hasattr(logger, 'warn'):
            logger.warn(
                f"Drone {drone_id}: cannot update yaw — no active target position"
            )
        else:
            logger.warning(
                f"Drone {drone_id}: cannot update yaw — no active target position"
            )
