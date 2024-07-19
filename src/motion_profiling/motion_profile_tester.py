import unittest
import numpy as np
from motion_profile_generator import generate_motion_profile, get_bezier_length

class TestMotionProfileGenerator(unittest.TestCase):
    def setUp(self):
        self.max_velocity = 10
        self.max_acceleration = 5
        self.max_jerk = 2

    def calculate_total_path_length(self, control_points):
        return sum(get_bezier_length(*cp) for cp in control_points)

    def test_simple_straight_line(self):
        control_points = [[[0, 0], [1, 0], [2, 0]]]
        target_velocities = [5]
        expected_length = self.calculate_total_path_length(control_points)
        
        positions, velocities, accelerations, times = generate_motion_profile(
            control_points, target_velocities, self.max_velocity, self.max_acceleration, self.max_jerk
        )

        # print("PUZISHINS: ", positions[-1], " ", expected_length)
        
        self.assertGreater(len(positions), 0)
        self.assertEqual(positions[0], 0)
        self.assertAlmostEqual(positions[-1], expected_length, places=2)
        self.assertLessEqual(max(velocities), self.max_velocity)
        self.assertLessEqual(max(np.abs(accelerations)), self.max_acceleration)

    def test_s_curve(self):
        control_points = [
            [[0, 0], [1, 1], [2, 0]],
            [[2, 0], [3, -1], [4, 0]]
        ]
        target_velocities = [5, 0]
        expected_length = self.calculate_total_path_length(control_points)
        
        positions, velocities, accelerations, times = generate_motion_profile(
            control_points, target_velocities, self.max_velocity, self.max_acceleration, self.max_jerk
        )
        
        self.assertGreater(len(positions), 0)
        self.assertEqual(positions[0], 0)
        self.assertAlmostEqual(positions[-1], expected_length, places=2)
        self.assertLessEqual(max(velocities), self.max_velocity)
        self.assertLessEqual(max(np.abs(accelerations)), self.max_acceleration)

    def test_complex_path(self):
        control_points = [
            [[0, 0], [1, 1], [2, 0]],
            [[2, 0], [3, -1], [4, 0], [5, 1]],
            [[5, 1], [6, 2], [7, 1]]
        ]
        target_velocities = [5, 2, 0]
        expected_length = self.calculate_total_path_length(control_points)
        
        positions, velocities, accelerations, times = generate_motion_profile(
            control_points, target_velocities, self.max_velocity, self.max_acceleration, self.max_jerk
        )
        
        self.assertGreater(len(positions), 0)
        self.assertEqual(positions[0], 0)
        self.assertAlmostEqual(positions[-1], expected_length, places=2)
        self.assertLessEqual(max(velocities), self.max_velocity)
        self.assertLessEqual(max(np.abs(accelerations)), self.max_acceleration)
        self.assertAlmostEqual(velocities[-1], 0, places=2)
        self.assertAlmostEqual(accelerations[-1], 0, places=2)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Run motion profile generator tests')
    parser.add_argument('--visualize', action='store_true', help='Visualize the motion profiles')
    args = parser.parse_args()

    # Run the tests
    suite = unittest.TestLoader().loadTestsFromTestCase(TestMotionProfileGenerator)
    unittest.TextTestRunner(verbosity=2).run(suite)

    # If visualization is requested, plot the results of the complex path test
    if args.visualize:
        from plot_motion_profile import plot_motion_profile
        
        test_case = TestMotionProfileGenerator()
        test_case.setUp()
        control_points = [
            [[0, 0], [1, 1], [2, 0]],
            [[2, 0], [3, -1], [4, 0], [5, 1]],
            [[5, 1], [6, 2], [7, 1]]
        ]
        target_velocities = [5, 2, 0]
        positions, velocities, accelerations, times = generate_motion_profile(
            control_points, target_velocities, test_case.max_velocity, test_case.max_acceleration, test_case.max_jerk
        )
        plot_motion_profile(positions, velocities, accelerations, times)