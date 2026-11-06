"""Shared speed factors for the existing physical motion profiles.

Speed and feedback frequency scale together. Ground acceleration scales with
the square of speed so the established braking distances remain available.
Spatial limits, route geometry and contact settling windows are unchanged.
"""
GROUND_SCALE = 2.0
CRANE_SCALE = 1.5


def ground_speed(value):
    return GROUND_SCALE * value


def ground_period(value):
    return value / GROUND_SCALE


def crane_speed(value):
    return CRANE_SCALE * value


def crane_period(value):
    return value / CRANE_SCALE
