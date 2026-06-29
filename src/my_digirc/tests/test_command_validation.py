"""
Tests for command validation logic in SwarmManager.
Run with: python -m pytest src/my_digirc/tests/
"""

import sys
import os

# Add the source directory to the path so we can import our modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from manager.commands import VALID_COMMANDS, validate_position, _clamp


class TestValidCommands:
    """Tests for the VALID_COMMANDS set."""

    def test_all_expected_commands_are_valid(self):
        expected = {"ARM", "TAKEOFF", "LAND", "GOTO", "START_OFFBOARD", "HOVER"}
        assert VALID_COMMANDS == expected

    def test_unknown_command_not_in_set(self):
        assert "FLY_TO_MOON" not in VALID_COMMANDS
        assert "arm" not in VALID_COMMANDS  # Case-sensitive check
        assert "" not in VALID_COMMANDS

    def test_valid_commands_is_a_set(self):
        """Ensure we're using a set for O(1) lookup."""
        assert isinstance(VALID_COMMANDS, set)


class TestClamp:
    """Tests for the _clamp utility."""

    def test_value_within_range(self):
        assert _clamp(5.0, 0.0, 10.0) == 5.0

    def test_value_below_min(self):
        assert _clamp(-5.0, 0.0, 10.0) == 0.0

    def test_value_above_max(self):
        assert _clamp(15.0, 0.0, 10.0) == 10.0

    def test_value_at_boundaries(self):
        assert _clamp(0.0, 0.0, 10.0) == 0.0
        assert _clamp(10.0, 0.0, 10.0) == 10.0


class TestValidatePosition:
    """Tests for position input validation."""

    def test_normal_values_pass_through(self):
        x, y, z = validate_position(10.0, 20.0, -5.0)
        assert x == 10.0
        assert y == 20.0
        assert z == -5.0

    def test_extreme_values_are_clamped(self):
        x, y, z = validate_position(9999.0, -9999.0, -9999.0)
        assert x == 500.0   # MAX_POSITION_M
        assert y == -500.0  # -MAX_POSITION_M
        assert z == -100.0  # -MAX_ALTITUDE_M

    def test_z_positive_clamped_to_near_ground(self):
        """In NED, positive z = below ground. Clamp to 10m underground max."""
        _, _, z = validate_position(0, 0, 50.0)
        assert z == 10.0

    def test_string_values_are_converted(self):
        """validate_position should handle string inputs gracefully."""
        x, y, z = validate_position("10.0", "20.0", "-5.0")
        assert x == 10.0
        assert y == 20.0
        assert z == -5.0
