#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point
import csv
import time

# Number of drones — configurable via environment variable
NUM_DRONES = int(os.environ.get("NUM_DRONES", "3"))

class SwarmTelemetryLogger(Node):
    def __init__(self):
        super().__init__("swarm_telemetry_logger")
        
        # Ensure the flight_logs directory exists
        log_dir = "flight_logs"
        os.makedirs(log_dir, exist_ok=True)
        
        # Open a new CSV file based on the current time
        filename = f"{log_dir}/swarm_flight_log_{int(time.time())}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # Write the header row
        self.csv_writer.writerow(["Timestamp", "DroneID", "X", "Y", "Z"])
        
        self.get_logger().info(f"Logging telemetry to {filename}.")

        # QoS: Best-effort to match the publisher's QoS
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to all drone telemetry topics
        for i in range(1, NUM_DRONES + 1):
            self.create_subscription(
                Point,
                f'/drone_{i}/telemetry',
                lambda msg, drone_id=i: self.log_telemetry(drone_id, msg),
                telemetry_qos
            )

    def log_telemetry(self, drone_id, msg):
        timestamp = time.time()
        self.csv_writer.writerow([timestamp, drone_id, msg.x, msg.y, msg.z])
        # We optionally flush here to ensure data is written if the node crashes
        self.csv_file.flush()

    def destroy_node(self):
        print("Closing log file...")
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SwarmTelemetryLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
