"""
SwarmManager module — split into focused sub-modules.

Re-exports SwarmManager for backward compatibility so existing code
can still do: `from manager import SwarmManager`
"""

from manager.manager import SwarmManager

__all__ = ["SwarmManager"]
