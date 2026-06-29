"""
Abstract base class for the Drone Hardware Abstraction Layer.

Think of this as a contract: any drone backend (simulation or real hardware)
must implement all of these methods. The rest of the code only calls these
methods and never talks to MAVSDK or serial ports directly.

This means switching from simulation to a real Pixhawk drone is just
swapping one class for another — no hunting through code to change things.
"""

from abc import ABC, abstractmethod


class AbstractDroneHAL(ABC):
    """
    Interface that every drone backend must implement.

    Each instance represents ONE physical (or simulated) drone.
    """

    @abstractmethod
    async def connect(self, connection_string: str) -> None:
        """
        Connect to the drone.

        Args:
            connection_string: How to reach the drone.
                Simulation: "udpin://0.0.0.0:14540"
                Real hardware: "serial:///dev/ttyUSB0:57600"
        """
        ...

    @abstractmethod
    async def wait_for_connection(self) -> None:
        """Block until the drone confirms it is connected."""
        ...

    @abstractmethod
    async def get_health(self):
        """
        Get the first health reading from the drone.

        Returns:
            An object with .is_global_position_ok and .is_home_position_ok
        """
        ...

    @abstractmethod
    async def arm(self) -> None:
        """Arm the drone's motors."""
        ...

    @abstractmethod
    async def hold(self) -> None:
        """Switch to HOLD/LOITER mode (hover in place using flight controller)."""
        ...

    @abstractmethod
    async def takeoff(self) -> None:
        """Command the drone to take off."""
        ...

    @abstractmethod
    async def land(self) -> None:
        """Command the drone to land."""
        ...

    @abstractmethod
    async def return_to_launch(self) -> None:
        """Command the drone to fly back to its launch position and land."""
        ...

    @abstractmethod
    async def start_offboard(self) -> None:
        """Start offboard (external computer) control mode."""
        ...

    @abstractmethod
    async def set_position_ned(self, north_m, east_m, down_m, yaw_deg) -> None:
        """
        Send a position setpoint in NED frame.

        Args:
            north_m: North position in metres
            east_m: East position in metres
            down_m: Down position in metres (negative = up)
            yaw_deg: Heading in degrees
        """
        ...

    @abstractmethod
    async def set_velocity_ned(self, north_ms, east_ms, down_ms, yaw_deg) -> None:
        """
        Send a velocity setpoint in NED frame.

        Args:
            north_ms: North velocity in m/s
            east_ms: East velocity in m/s
            down_ms: Down velocity in m/s
            yaw_deg: Heading in degrees
        """
        ...

    @abstractmethod
    def position_velocity_ned_stream(self):
        """
        Return an async iterator that yields position/velocity updates.
        Each yielded item should have a .position with .north_m, .east_m, .down_m
        """
        ...

    @abstractmethod
    def battery_stream(self):
        """
        Return an async iterator that yields battery updates.
        Each yielded item should have a .remaining_percent field (0.0 to 1.0).
        """
        ...

    @abstractmethod
    def health_stream(self):
        """
        Return an async iterator that yields health updates.
        Each yielded item should have .is_global_position_ok and .is_home_position_ok
        """
        ...
