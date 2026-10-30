#!/usr/bin/env python3
"""Regression checks for the E2E observer's independent DDS startup ordering."""

import math
import unittest

from test_tunnel_navigation import initial_joint_feedback_ready


class ObserverBootstrapTests(unittest.TestCase):
    def test_guard_discovery_does_not_substitute_for_joint_discovery(self):
        for lift, pitch in [(None, None), (None, 0.0), (0.055, None)]:
            self.assertFalse(initial_joint_feedback_ready(lift, pitch, 1.0, 1.1, 0.5))

    def test_stale_and_future_feedback_are_not_ready(self):
        self.assertFalse(initial_joint_feedback_ready(0.055, 0.0, 1.0, 1.6, 0.5))
        self.assertFalse(initial_joint_feedback_ready(0.055, 0.0, 1.1, 1.0, 0.5))

    def test_nonfinite_joint_positions_are_not_ready(self):
        for lift, pitch in [(math.nan, 0.0), (0.055, math.inf)]:
            self.assertFalse(initial_joint_feedback_ready(lift, pitch, 1.0, 1.1, 0.5))

    def test_fresh_normal_and_low_postures_can_start_observation(self):
        # Normal height must not be hidden: the separate inside-roof assertion
        # must fail if a resumed robot is high. Bootstrap only checks discovery.
        for lift in (0.0, 0.055):
            self.assertTrue(initial_joint_feedback_ready(lift, 0.0, 1.0, 1.1, 0.5))


if __name__ == "__main__":
    unittest.main()
