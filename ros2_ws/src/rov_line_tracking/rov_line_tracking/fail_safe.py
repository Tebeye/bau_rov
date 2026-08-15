#!/usr/bin/env python3
"""
==============================================================================
FAZ 5: Fail-Safe Mechanism & Dead Reckoning State Machine
==============================================================================
Task Requirements:
- T-5.1: LostCounter logic & "Search Mode (Dead Reckoning)" state.
- T-5.2: Emergency stop horizontal motors & positive Heave surface ascent trigger.
==============================================================================
"""

import math
from enum import Enum


class SystemState(Enum):
    LINE_FOLLOWING = 1
    SEARCH_MODE = 2
    EMERGENCY_SURFACE = 3


class FailSafeManager:
    def __init__(self, timeout_count: int = 30, emergency_ascent_force: float = 5.0):
        """
        :param timeout_count: Number of consecutive NaN frames before emergency surface ascent (e.g. 30 ticks = 3 sec at 10Hz).
        :param emergency_ascent_force: Positive upward thrust (N) for emergency surfacing.
        """
        self.timeout_count = timeout_count
        self.emergency_ascent_force = emergency_ascent_force

        self.lost_counter = 0
        self.search_phase_ticks = 0
        self.state = SystemState.LINE_FOLLOWING

    def update(self, line_error: float) -> SystemState:
        """Updates the state machine based on the current vision line error measurement."""
        if math.isnan(line_error):
            self.lost_counter += 1
            self.search_phase_ticks += 1

            if self.lost_counter > self.timeout_count:
                self.state = SystemState.EMERGENCY_SURFACE
            else:
                self.state = SystemState.SEARCH_MODE
        else:
            # Line re-acquired
            self.lost_counter = 0
            self.search_phase_ticks = 0
            self.state = SystemState.LINE_FOLLOWING

        return self.state

    def get_search_pattern_wrench(self, nominal_surge: float = 2.0):
        """
        Generates a sweeping Search Pattern (Dead Reckoning) Wrench command.
        Slow forward surge with sinusoidal yaw sweep to relocate the line.
        """
        # Sinusoidal search sweep for Yaw (torque)
        yaw_torque = 1.5 * math.sin(self.search_phase_ticks * 0.2)
        surge_force = nominal_surge * 0.5  # Half speed search
        sway_force = 0.0
        return surge_force, sway_force, yaw_torque

    def get_emergency_surface_wrench(self):
        """
        T-5.2: Emergency procedure:
        Stops horizontal thrusters (surge=0, sway=0, yaw=0) and applies positive Heave (upward force).
        """
        return 0.0, 0.0, 0.0, self.emergency_ascent_force
