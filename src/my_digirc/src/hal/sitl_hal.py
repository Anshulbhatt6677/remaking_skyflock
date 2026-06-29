"""
SITL (Software-In-The-Loop) HAL implementation.

This wraps MAVSDK calls — exactly the code that was already running
in swarm_manager.py — behind the AbstractDroneHAL interface.
Used for Gazebo simulation.
"""

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw

from hal.abstract_hal import AbstractDroneHAL


class SitlHAL(AbstractDroneHAL):
    """
    MAVSDK-based drone backend for PX4 SITL simulation.

    Usage:
        hal = SitlHAL(grpc_port=50051)
        await hal.connect("udpin://0.0.0.0:14540")
        await hal.wait_for_connection()
        await hal.arm()
    """

    def __init__(self, grpc_port=50051):
        """
        Args:
            grpc_port: The gRPC port for the mavsdk_server process.
                       Each drone needs a unique port.
        """
        self._system = System(port=grpc_port)

    @property
    def system(self):
        """Access the underlying MAVSDK System object (for advanced use)."""
        return self._system

    async def connect(self, connection_string: str) -> None:
        await self._system.connect(system_address=connection_string)

    async def wait_for_connection(self) -> None:
        async for state in self._system.core.connection_state():
            if state.is_connected:
                break

    async def get_health(self):
        async for health in self._system.telemetry.health():
            return health

    async def arm(self) -> None:
        await self._system.action.arm()

    async def hold(self) -> None:
        await self._system.action.hold()

    async def takeoff(self) -> None:
        await self._system.action.takeoff()

    async def land(self) -> None:
        await self._system.action.land()

    async def return_to_launch(self) -> None:
        await self._system.action.return_to_launch()

    async def start_offboard(self) -> None:
        await self._system.offboard.start()

    async def set_position_ned(self, north_m, east_m, down_m, yaw_deg) -> None:
        await self._system.offboard.set_position_ned(
            PositionNedYaw(north_m, east_m, down_m, yaw_deg)
        )

    async def set_velocity_ned(self, north_ms, east_ms, down_ms, yaw_deg) -> None:
        await self._system.offboard.set_velocity_ned(
            VelocityNedYaw(north_ms, east_ms, down_ms, yaw_deg)
        )

    def position_velocity_ned_stream(self):
        return self._system.telemetry.position_velocity_ned()

    def battery_stream(self):
        return self._system.telemetry.battery()

    def health_stream(self):
        return self._system.telemetry.health()
