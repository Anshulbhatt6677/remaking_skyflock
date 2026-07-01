#!/usr/bin/env python3

import asyncio
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from my_digirc.srv import Command
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

from manager import SwarmManager


import json
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

try:
    from prometheus_client import start_http_server, Gauge, Counter
    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False


# Number of drones — configurable via environment variable
NUM_DRONES = int(os.environ.get("NUM_DRONES", "3"))

class JsonLoggerWrapper:
    """Wraps a ROS 2 logger to emit structured JSON logs."""
    def __init__(self, ros_logger):
        self.logger = ros_logger

    def _log(self, level, msg):
        if level == "INFO":
            self.logger.info(msg)
        elif level in ["WARN", "WARNING"]:
            self.logger.warning(msg)
        elif level == "ERROR":
            self.logger.error(msg)
        elif level == "DEBUG":
            self.logger.debug(msg)

    def debug(self, msg): self._log("DEBUG", msg)
    def info(self, msg): self._log("INFO", msg)
    def warning(self, msg): self._log("WARN", msg)
    def warn(self, msg): self._log("WARN", msg)
    def error(self, msg): self._log("ERROR", msg)


class SwarmController(Node):
    """
    ROS 2 node that bridges service/topic calls to the SwarmManager.
    Now with Observability: Prometheus metrics and ROS 2 Diagnostics.
    """

    def __init__(self):
        super().__init__("swarm_controller")

        # Start Prometheus metrics server
        self.prometheus_enabled = PROMETHEUS_AVAILABLE
        if self.prometheus_enabled:
            self.get_logger().info("Starting Prometheus metrics server on port 8000...")
            try:
                start_http_server(8000)
                self.metrics_altitude = Gauge('drone_altitude_m', 'Live drone altitude', ['drone_id'])
                self.metrics_battery = Gauge('drone_battery_percent', 'Live battery percentage', ['drone_id'])
                self.metrics_commands = Counter('swarm_commands_total', 'Total executed commands', ['drone_id', 'command'])
            except Exception as e:
                self.get_logger().error(f"Failed to start Prometheus: {e}")
                self.prometheus_enabled = False

        # Diagnostics publisher
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Use JSON structured logging
        self.json_logger = JsonLoggerWrapper(self.get_logger())
        self.swarm = SwarmManager(logger=self.json_logger)

        for i in range(1, NUM_DRONES + 1):
            self.swarm.add_drone(i, 14540 + (i - 1))

        # QoS Profiles
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        yaw_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Telemetry publishers
        self.telemetry_pubs = {
            i: self.create_publisher(Point, f"/drone_{i}/telemetry", telemetry_qos)
            for i in range(1, NUM_DRONES + 1)
        }
        
        # Register callbacks
        self.swarm.register_telemetry_callback(self.telemetry_cb)
        self.swarm.register_battery_callback(self.battery_cb)
        self.swarm.register_health_callback(self.health_cb)

        # Asyncio event loop on a background thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(
            target=self.loop.run_forever,
            daemon=True,
        ).start()

        # Connect all drones asynchronously
        asyncio.run_coroutine_threadsafe(
            self.swarm.connect_all_drones(),
            self.loop,
        )

        # Per-drone services and subscribers
        self.drone_services = {}
        for i in range(1, NUM_DRONES + 1):
            self.drone_services[i] = self.create_service(
                Command,
                f"/drone_{i}/mission_command",
                lambda req, res, drone_id=i: self.handle_command(req, res, drone_id),
            )

            self.create_subscription(
                Float32,
                f"/drone_{i}/yaw_command",
                lambda msg, drone_id=i: self.yaw_cb(drone_id, msg),
                yaw_qos,
            )

        self.get_logger().info(f"SwarmController ready — managing {NUM_DRONES} drones")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def telemetry_cb(self, drone_id, x, y, z):
        """Forward telemetry to ROS topic and Prometheus."""
        msg = Point()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        self.telemetry_pubs[drone_id].publish(msg)
        
        if self.prometheus_enabled:
            self.metrics_altitude.labels(drone_id=str(drone_id)).set(z)

    def battery_cb(self, drone_id, battery_percent):
        """Update Prometheus with battery level."""
        if self.prometheus_enabled:
            self.metrics_battery.labels(drone_id=str(drone_id)).set(battery_percent * 100.0)

    def health_cb(self, drone_id, global_ok, home_ok):
        """Publish ROS 2 Diagnostics based on health."""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = f"drone_{drone_id}_health"
        status.hardware_id = f"drone_{drone_id}"
        
        if global_ok and home_ok:
            status.level = DiagnosticStatus.OK
            status.message = "GPS and Home position OK"
        else:
            status.level = DiagnosticStatus.ERROR
            status.message = "Missing GPS or Home position"
            
        diag_array.status.append(status)
        self.diag_pub.publish(diag_array)

    def yaw_cb(self, drone_id, msg):
        """Forward yaw commands from a ROS topic to SwarmManager."""
        self.swarm.update_yaw(drone_id, msg.data)

    def handle_command(self, request, response, drone_id):
        """Service handler for /drone_N/mission_command."""
        # Only log non-GOTO commands to avoid spam during continuous tick updates
        if request.command != "GOTO":
            self.json_logger.info(
                f"Drone {drone_id}: received command={request.command} "
                f"x={request.x} y={request.y} z={request.z} yaw={request.yaw}"
            )
        
        if self.prometheus_enabled:
            self.metrics_commands.labels(drone_id=str(drone_id), command=request.command).inc()

        asyncio.run_coroutine_threadsafe(
            self.swarm.execute_command(
                drone_id,
                request.command,
                request.x,
                request.y,
                request.z,
                request.yaw,
            ),
            self.loop,
        )

        response.response = f"Command {request.command} accepted"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
