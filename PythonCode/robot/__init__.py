"""The robot: hardware, behaviours, the loop between them.

A package rather than a thousand-line module. The split is by job, not by
size: what the car is told about itself (aiming), what it tells the world
(telemetry), and what it does (core).
"""

from .core import Robot

__all__ = ["Robot"]
