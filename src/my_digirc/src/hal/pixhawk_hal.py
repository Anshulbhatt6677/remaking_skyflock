"""
Pixhawk (real hardware) HAL implementation — STUB.

This is a placeholder for when you connect to a real Pixhawk
flight controller over serial/USB instead of a Gazebo simulation.

The interface is identical to SitlHAL, but the connection string
and some failsafe parameters will differ. For now, it raises
NotImplementedError so you know which methods need filling in.
"""

from hal.abstract_hal import AbstractDroneHAL


class PixhawkHAL(AbstractDroneHAL):
    """
    Real-hardware drone backend for Pixhawk over serial.

    STUB — not yet implemented. When you're ready to fly real drones:
    1. Copy SitlHAL as a starting point
    2. Change the connection string to serial (e.g. "serial:///dev/ttyUSB0:57600")
    3. Add hardware-specific failsafe parameters
    4. Add RC-loss handling
    """

    def __init__(self, grpc_port=50051):
        self._grpc_port = grpc_port

    async def connect(self, connection_string: str) -> None:
        raise NotImplementedError(
            "PixhawkHAL.connect() not yet implemented. "
            "Use SitlHAL for simulation."
        )

    async def wait_for_connection(self) -> None:
        raise NotImplementedError("PixhawkHAL.wait_for_connection() not yet implemented.")

    async def get_health(self):
        raise NotImplementedError("PixhawkHAL.get_health() not yet implemented.")

    async def arm(self) -> None:
        raise NotImplementedError("PixhawkHAL.arm() not yet implemented.")

    async def hold(self) -> None:
        raise NotImplementedError("PixhawkHAL.hold() not yet implemented.")

    async def takeoff(self) -> None:
        raise NotImplementedError("PixhawkHAL.takeoff() not yet implemented.")

    async def land(self) -> None:
        raise NotImplementedError("PixhawkHAL.land() not yet implemented.")

    async def return_to_launch(self) -> None:
        raise NotImplementedError("PixhawkHAL.return_to_launch() not yet implemented.")

    async def start_offboard(self) -> None:
        raise NotImplementedError("PixhawkHAL.start_offboard() not yet implemented.")

    async def set_position_ned(self, north_m, east_m, down_m, yaw_deg) -> None:
        raise NotImplementedError("PixhawkHAL.set_position_ned() not yet implemented.")

    async def set_velocity_ned(self, north_ms, east_ms, down_ms, yaw_deg) -> None:
        raise NotImplementedError("PixhawkHAL.set_velocity_ned() not yet implemented.")

    def position_velocity_ned_stream(self):
        raise NotImplementedError("PixhawkHAL.position_velocity_ned_stream() not yet implemented.")

    def battery_stream(self):
        raise NotImplementedError("PixhawkHAL.battery_stream() not yet implemented.")

    def health_stream(self):
        raise NotImplementedError("PixhawkHAL.health_stream() not yet implemented.")
