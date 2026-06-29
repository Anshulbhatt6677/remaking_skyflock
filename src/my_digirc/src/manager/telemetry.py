"""
Telemetry and battery monitoring for MAVSDK drones.

Background async loops that stream position data and monitor battery levels.
"""

BATTERY_CRITICAL_THRESHOLD = 0.20  # 20%


async def telemetry_loop(drones, drone_id, telemetry_callbacks, logger):
    """
    Stream NED position from PX4 and forward to registered callbacks.
    Runs indefinitely as a background task.
    """
    drone = drones[drone_id]["system"]
    error_count = 0

    try:
        async for pos_vel in drone.telemetry.position_velocity_ned():
            pos = pos_vel.position
            for cb in telemetry_callbacks:
                try:
                    cb(drone_id, pos.north_m, pos.east_m, pos.down_m)
                except Exception as e:
                    error_count += 1
                    # Log every 100th error to avoid spam
                    if error_count % 100 == 1:
                        if hasattr(logger, 'warn'):
                            logger.warn(
                                f"Drone {drone_id}: telemetry callback error "
                                f"(count={error_count}): {e}"
                            )
                        else:
                            logger.warning(
                                f"Drone {drone_id}: telemetry callback error "
                                f"(count={error_count}): {e}"
                            )
    except Exception as e:
        logger.info(f"Drone {drone_id}: telemetry loop stopped: {e}")


async def battery_monitor_loop(drones, drone_id, logger, battery_callbacks=None):
    """
    Monitor battery and trigger RTL if critically low.
    Runs indefinitely as a background task; exits after triggering RTL.
    """
    drone = drones[drone_id]["system"]

    try:
        async for battery in drone.telemetry.battery():
            if battery_callbacks:
                for cb in battery_callbacks:
                    try:
                        cb(drone_id, battery.remaining_percent)
                    except Exception as e:
                        logger.warning(f"Drone {drone_id}: battery callback error: {e}")

            if battery.remaining_percent < BATTERY_CRITICAL_THRESHOLD:
                logger.error(
                    f"CRITICAL: Drone {drone_id} battery low "
                    f"({battery.remaining_percent * 100:.1f}%). Triggering RTL!"
                )

                # Cancel the streaming task so it doesn't fight RTL mode
                task = drones[drone_id].get("setpoint_task")
                if task is not None:
                    task.cancel()
                    drones[drone_id]["setpoint_task"] = None

                try:
                    await drone.action.return_to_launch()
                    logger.info(f"Drone {drone_id}: RTL triggered successfully")
                except Exception as e:
                    logger.error(f"Drone {drone_id}: RTL failed — {e}")

                break  # Exit the monitor loop once RTL is triggered
    except Exception as e:
        logger.info(f"Drone {drone_id}: battery loop stopped: {e}")

async def health_monitor_loop(drones, drone_id, logger, health_callbacks=None):
    """
    Monitor GPS and EKF health continuously for diagnostics.
    Runs indefinitely as a background task.
    """
    drone = drones[drone_id]["system"]
    
    try:
        async for health in drone.telemetry.health():
            drones[drone_id]["is_global_position_ok"] = health.is_global_position_ok
            drones[drone_id]["is_home_position_ok"] = health.is_home_position_ok

            if health_callbacks:
                for cb in health_callbacks:
                    try:
                        cb(drone_id, health.is_global_position_ok, health.is_home_position_ok)
                    except Exception as e:
                        logger.warning(f"Drone {drone_id}: health callback error: {e}")
    except Exception as e:
        logger.info(f"Drone {drone_id}: health loop stopped: {e}")

async def home_monitor_loop(drones, drone_id, logger):
    """
    Fetch the absolute home altitude once it is established.
    Used for altitude synchronization across the swarm.
    """
    drone = drones[drone_id]["system"]
    try:
        async for home in drone.telemetry.home():
            if "home_alt" not in drones[drone_id]:
                logger.info(f"Drone {drone_id}: initial home altitude locked at {home.absolute_altitude_m:.2f}m")
            drones[drone_id]["home_alt"] = home.absolute_altitude_m
    except Exception as e:
        logger.info(f"Drone {drone_id}: home loop stopped: {e}")
