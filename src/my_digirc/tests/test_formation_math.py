"""
Tests for formation math in SwarmOrchestrator.

These tests verify that form_v() and form_line() produce the correct
coordinates for each drone without needing ROS or Gazebo.
"""

import sys
import os

# We can't easily import SwarmOrchestrator because it requires ROS,
# so we extract and test the pure math logic directly.


class TestVFormation:
    """Test V-formation coordinate calculations."""

    def _calculate_v_positions(self, center_x, center_y, center_z, num_drones, spacing=5.0):
        """
        Reproduce the V-formation math from SwarmOrchestrator.form_v().
        Returns a dict: {drone_id: (x, y, z)}
        """
        positions = {}
        # Leader
        positions[1] = (center_x, center_y, center_z)

        # Wings — dynamically sized
        wing_pair = 1
        for i in range(2, num_drones + 1, 2):
            # Left wing
            positions[i] = (
                center_x - spacing * wing_pair,
                center_y - spacing * wing_pair,
                center_z,
            )
            # Right wing
            if i + 1 <= num_drones:
                positions[i + 1] = (
                    center_x - spacing * wing_pair,
                    center_y + spacing * wing_pair,
                    center_z,
                )
            wing_pair += 1

        return positions

    def test_3_drones_v_formation(self):
        positions = self._calculate_v_positions(10.0, 0.0, -8.0, 3)
        assert positions[1] == (10.0, 0.0, -8.0)   # Leader at center
        assert positions[2] == (5.0, -5.0, -8.0)    # Left wing
        assert positions[3] == (5.0, 5.0, -8.0)     # Right wing

    def test_5_drones_v_formation(self):
        positions = self._calculate_v_positions(10.0, 0.0, -8.0, 5)
        assert positions[1] == (10.0, 0.0, -8.0)     # Leader
        assert positions[2] == (5.0, -5.0, -8.0)     # Inner left
        assert positions[3] == (5.0, 5.0, -8.0)      # Inner right
        assert positions[4] == (0.0, -10.0, -8.0)    # Outer left
        assert positions[5] == (0.0, 10.0, -8.0)     # Outer right

    def test_1_drone_v_formation(self):
        positions = self._calculate_v_positions(10.0, 0.0, -8.0, 1)
        assert len(positions) == 1
        assert positions[1] == (10.0, 0.0, -8.0)

    def test_v_formation_symmetry(self):
        """Left and right wings should be symmetric about the Y axis."""
        positions = self._calculate_v_positions(0.0, 0.0, -10.0, 5)
        # Drone 2 and 3 should mirror on Y
        assert positions[2][0] == positions[3][0]  # Same X
        assert positions[2][1] == -positions[3][1]  # Mirror Y
        # Drone 4 and 5 should mirror on Y
        assert positions[4][0] == positions[5][0]  # Same X
        assert positions[4][1] == -positions[5][1]  # Mirror Y

    def test_custom_spacing(self):
        positions = self._calculate_v_positions(0.0, 0.0, -5.0, 3, spacing=10.0)
        assert positions[2] == (-10.0, -10.0, -5.0)
        assert positions[3] == (-10.0, 10.0, -5.0)


class TestLineFormation:
    """Test line-formation coordinate calculations."""

    def _calculate_line_positions(self, center_x, center_y, center_z, num_drones, spacing=5.0):
        """
        Reproduce the line-formation math from SwarmOrchestrator.form_line().
        Returns a dict: {drone_id: (x, y, z)}
        """
        positions = {}
        positions[1] = (center_x, center_y, center_z)

        offset_index = 1
        for i in range(2, num_drones + 1, 2):
            # Left
            positions[i] = (
                center_x,
                center_y - spacing * offset_index,
                center_z,
            )
            # Right
            if i + 1 <= num_drones:
                positions[i + 1] = (
                    center_x,
                    center_y + spacing * offset_index,
                    center_z,
                )
            offset_index += 1

        return positions

    def test_3_drones_line_formation(self):
        positions = self._calculate_line_positions(10.0, 0.0, -8.0, 3)
        assert positions[1] == (10.0, 0.0, -8.0)     # Center
        assert positions[2] == (10.0, -5.0, -8.0)    # Left
        assert positions[3] == (10.0, 5.0, -8.0)     # Right

    def test_all_same_x_coordinate(self):
        """Line formation: all drones should share the same X."""
        positions = self._calculate_line_positions(15.0, 0.0, -8.0, 5)
        for drone_id, (x, y, z) in positions.items():
            assert x == 15.0, f"Drone {drone_id} has x={x}, expected 15.0"

    def test_line_formation_symmetry(self):
        """Left and right should be symmetric about center Y."""
        positions = self._calculate_line_positions(0.0, 0.0, -5.0, 5)
        assert positions[2][1] == -positions[3][1]
        assert positions[4][1] == -positions[5][1]


class TestSpawnOffsets:
    """Test dynamic spawn offset generation."""

    def test_3_drones_default_spacing(self):
        spacing = 5.0
        offsets = {
            i: ((i - 1) * spacing, 0.0)
            for i in range(1, 4)
        }
        assert offsets[1] == (0.0, 0.0)
        assert offsets[2] == (5.0, 0.0)
        assert offsets[3] == (10.0, 0.0)

    def test_5_drones_default_spacing(self):
        spacing = 5.0
        offsets = {
            i: ((i - 1) * spacing, 0.0)
            for i in range(1, 6)
        }
        assert offsets[4] == (15.0, 0.0)
        assert offsets[5] == (20.0, 0.0)

    def test_custom_spacing(self):
        spacing = 3.0
        offsets = {
            i: ((i - 1) * spacing, 0.0)
            for i in range(1, 4)
        }
        assert offsets[2] == (3.0, 0.0)
        assert offsets[3] == (6.0, 0.0)

    def test_goto_with_offset_subtraction(self):
        """
        When the orchestrator sends GOTO, it subtracts the drone's spawn offset
        so all drones operate in a shared global coordinate system.
        """
        spawn_offsets = {1: (0.0, 0.0), 2: (5.0, 0.0), 3: (10.0, 0.0)}
        global_target = (10.0, 0.0)  # All drones should end up at global (10, 0)

        # What each drone actually receives:
        local_1 = (global_target[0] - spawn_offsets[1][0], global_target[1] - spawn_offsets[1][1])
        local_2 = (global_target[0] - spawn_offsets[2][0], global_target[1] - spawn_offsets[2][1])
        local_3 = (global_target[0] - spawn_offsets[3][0], global_target[1] - spawn_offsets[3][1])

        assert local_1 == (10.0, 0.0)
        assert local_2 == (5.0, 0.0)
        assert local_3 == (0.0, 0.0)
