import unittest
import math
from rov_line_tracking.fail_safe import FailSafeManager, SystemState
from rov_line_tracking.control_node import PIDController


class TestROVControl(unittest.TestCase):

    def test_pid_clamping(self):
        pid = PIDController(kp=2.0, ki=0.5, kd=0.1, max_output=5.0, dt=0.1)
        output = pid.compute(target=2.0, current=0.0)
        self.assertLessEqual(output, 5.0)
        self.assertGreaterEqual(output, -5.0)

    def test_fail_safe_transitions(self):
        manager = FailSafeManager(timeout_count=5, emergency_ascent_force=5.0)

        # Line detected
        state = manager.update(line_error=10.5)
        self.assertEqual(state, SystemState.LINE_FOLLOWING)

        # Line lost for 3 frames (Search mode)
        for _ in range(3):
            state = manager.update(line_error=float('nan'))
        self.assertEqual(state, SystemState.SEARCH_MODE)

        # Line lost for 3 more frames (Timeout exceeded -> Emergency Surface)
        for _ in range(3):
            state = manager.update(line_error=float('nan'))
        self.assertEqual(state, SystemState.EMERGENCY_SURFACE)


if __name__ == '__main__':
    unittest.main()
