"""
Flight commands for MAVSDK drones.

Each function operates on the shared `drones` state dict and the
MAVSDK System object within it.
"""

import asyncio
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw


# Safety limits
MAX_POSITION_M = 500.0    # Max absolute coordinate value (metres)
MAX_ALTITUDE_M = 100.0    # Max altitude (NED down, so this is the min z value)

# Valid commands accepted by execute_command()
VALID_COMMANDS = {"ARM", "TAKEOFF", "LAND", "GOTO", "START_OFFBOARD", "HOVER"}


def _clamp(value, min_val, max_val):
    """Clamp a value to [min_val, max_val]."""
    return max(min_val, min(max_val, value))


def validate_position(x, y, z):
    """Clamp position values to safe ranges and return (x, y, z)."""
    x = _clamp(float(x), -MAX_POSITION_M, MAX_POSITION_M)
    y = _clamp(float(y), -MAX_POSITION_M, MAX_POSITION_M)
    # NED: negative z = up, so clamp to [-MAX_ALTITUDE, some_positive_value]
    z = _clamp(float(z), -MAX_ALTITUDE_M, 10.0)
    return x, y, z


async def arm_drone(drones, drone_id, logger):
    """Arm a single drone."""
    drone = drones[drone_id]["system"]
    try:
        await drone.action.arm()
        logger.info(f"Drone {drone_id}: armed")
    except Exception as e:
        logger.error(f"Drone {drone_id}: ARM ERROR — {e}")


async def takeoff_drone(drones, drone_id, logger):
    """Run the full takeoff sequence: health check → hold → arm → takeoff."""
    drone = drones[drone_id]["system"]
    logger.info(f"Drone {drone_id}: starting takeoff sequence")

    # Wait for GPS/health — with a timeout so failures are visible
    logger.info(f"Drone {drone_id}: waiting for EKF health lock...")
    health_wait_start = __import__('time').time()
    health_wait_timeout = 120  # seconds
    check_count = 0
    while not (drones[drone_id].get("is_global_position_ok") and drones[drone_id].get("is_home_position_ok")):
        elapsed = __import__('time').time() - health_wait_start
        if elapsed > health_wait_timeout:
            gps_ok = drones[drone_id].get("is_global_position_ok", False)
            home_ok = drones[drone_id].get("is_home_position_ok", False)
            logger.error(
                f"Drone {drone_id}: TAKEOFF ABORTED — EKF did not converge after "
                f"{health_wait_timeout}s. global_pos={gps_ok}, home_pos={home_ok}. "
                f"This is a Gazebo simulation speed issue — the drone spawned too late "
                f"for its EKF to fully initialize. Try increasing the sleep delays in "
                f"start_swarm.sh or reducing NUM_DRONES."
            )
            return
        check_count += 1
        if check_count % 5 == 1:  # Log every 10 seconds (5 * 2s sleep)
            gps_ok = drones[drone_id].get("is_global_position_ok", False)
            home_ok = drones[drone_id].get("is_home_position_ok", False)
            logger.info(
                f"Drone {drone_id}: still waiting for EKF — "
                f"global_pos={gps_ok}, home_pos={home_ok} "
                f"({elapsed:.0f}s elapsed)"
            )
        await asyncio.sleep(2)

    logger.info(f"Drone {drone_id}: health OK — proceeding to takeoff")

    # Switch to hold to clear any stale land state
    try:
        logger.info(f"Drone {drone_id}: switching to HOLD mode")
        await drone.action.hold()
        await asyncio.sleep(1)
    except Exception as e:
        if hasattr(logger, 'warn'):
            logger.warn(f"Drone {drone_id}: HOLD warning — {e}")
        else:
            logger.warning(f"Drone {drone_id}: HOLD warning — {e}")

    # Set takeoff altitude.
    # IMPORTANT: PX4's set_takeoff_altitude() is RELATIVE to each drone's own
    # home position — not an absolute MSL value. Setting it to 8.0 means
    # "climb 8 metres above where you are sitting", which is correct for all
    # drones regardless of where they spawned.
    try:
        takeoff_alt = 8.0
        logger.info(f"Drone {drone_id}: setting takeoff altitude to {takeoff_alt}m (relative to home)")
        await drone.action.set_takeoff_altitude(takeoff_alt)
    except Exception as e:
        if hasattr(logger, 'warn'):
            logger.warn(f"Drone {drone_id}: could not set takeoff altitude — {e}")
        else:
            logger.warning(f"Drone {drone_id}: could not set takeoff altitude — {e}")

    # Arm (retry loop for slow transitions after landing)
    armed_successfully = False
    for attempt in range(3):
        try:
            logger.info(f"Drone {drone_id}: arming (attempt {attempt+1})")
            await drone.action.arm()
            await asyncio.sleep(1)
            armed_successfully = True
            break
        except Exception as e:
            msg = f"Drone {drone_id}: ARM warning — {e}"
            if "COMMAND_DENIED" in str(e):
                logger.info(f"Drone {drone_id}: already armed.")
                armed_successfully = True
                break
            
            if hasattr(logger, 'warn'):
                logger.warn(msg)
            else:
                logger.warning(msg)
            await asyncio.sleep(1)
            
    if not armed_successfully:
        logger.error(f"Drone {drone_id}: Failed to arm after 3 attempts.")

    # Takeoff
    try:
        logger.info(f"Drone {drone_id}: sending takeoff command")
        await drone.action.takeoff()
        logger.info(f"Drone {drone_id}: takeoff command sent")
    except Exception as e:
        logger.error(f"Drone {drone_id}: TAKEOFF ERROR — {e}")


async def land_drone(drones, drone_id, logger):
    """Cancel setpoint streaming and land."""
    task = drones[drone_id].get("setpoint_task")
    if task is not None:
        task.cancel()
        drones[drone_id]["setpoint_task"] = None

    drone = drones[drone_id]["system"]
    logger.info(f"Drone {drone_id}: landing")
    await drone.action.land()
    logger.info(f"Drone {drone_id}: land command sent")


async def goto_position(drones, drone_id, x, y, z, yaw, logger):
    """
    Update the target position for the setpoint streaming loop.

    Offboard GOTO uses NED coordinates local to each drone's origin.
    Since drones spawn at different X positions (5m apart), but all at the
    same ground height, their NED Z origins are the same — no altitude
    offset correction is needed for Z in offboard mode.
    """
    x, y, z = validate_position(x, y, z)
    logger.info(f"Drone {drone_id}: GOTO x={x} y={y} z={z} yaw={yaw}")
    drones[drone_id]["target_position"] = PositionNedYaw(x, y, z, yaw)


async def start_offboard(drones, drone_id, logger, setpoint_loop_fn):
    """Prime setpoints and start offboard mode + background streaming."""
    drone = drones[drone_id]["system"]
    logger.info(f"Drone {drone_id}: starting offboard mode")

    # Prime with current target or hover
    target = drones[drone_id].get("target_position")
    if target is not None:
        await drone.offboard.set_position_ned(target)
    else:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )

    await drone.offboard.start()
    logger.info(f"Drone {drone_id}: offboard started")

    # Start the background setpoint streaming task
    if drones[drone_id].get("setpoint_task") is None:
        drones[drone_id]["setpoint_task"] = asyncio.create_task(
            setpoint_loop_fn(drone_id)
        )


async def hover_drone(drones, drone_id, logger):
    """Force hover by clearing the target position."""
    logger.info(f"Drone {drone_id}: hovering in place")
    drones[drone_id]["target_position"] = None


async def check_health(drones, drone_id, logger):
    """Print the current health status of a drone."""
    global_ok = drones[drone_id].get("is_global_position_ok", False)
    home_ok = drones[drone_id].get("is_home_position_ok", False)
    logger.info(
        f"Drone {drone_id}: "
        f"global_pos={global_ok}, "
        f"home_pos={home_ok}"
    )
