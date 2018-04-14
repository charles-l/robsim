#!/bin/env python3
import robsim
import sys
import numpy as np
import matplotlib.pyplot as plt
import unittest

class TestOnPath(unittest.TestCase):
    PLOT_ON_TEARDOWN = False

    def setUp(self):
        plt.figure()
        plt.axis('equal')
        robsim.plot_line_map()

    def tearDown(self):
        if self.PLOT_ON_TEARDOWN:
            plt.show()

    def plot_and_assert(self, p, on_path = True):
        try:
            if on_path:
                self.assertTrue(robsim.on_path(p))
            else:
                self.assertFalse(robsim.on_path(p))
        except AssertionError:
            plt.plot(p[0], p[1], 'xr')
            plt.show()
            raise
        else:
            plt.plot(p[0], p[1], on_path and 'ok' or 'sb')

    def test_upper_convex(self):
        self.plot_and_assert(np.array([0, 4]))
        self.plot_and_assert(np.array([-2, 2]))
        self.plot_and_assert(np.array([2, 2]))
        self.plot_and_assert(np.array([1.414, 3.414]))
        self.plot_and_assert(np.array([1, 4]))
        self.plot_and_assert(np.array([2, 3]))
        self.plot_and_assert(np.array([-2.2, 2]))
        self.plot_and_assert(np.array([-1.84, 2]))
        self.plot_and_assert(np.array([-2, 4]), on_path = False)
        self.plot_and_assert(np.array([1, 3]), on_path = False)
        self.plot_and_assert(np.array([-1, 3]), on_path = False)

    def test_lower_concave(self):
        self.plot_and_assert(np.array([0, -4]))
        self.plot_and_assert(np.array([-2, -2]))
        self.plot_and_assert(np.array([-2, -2]))
        self.plot_and_assert(np.array([1.414, -3.414]))
        self.plot_and_assert(np.array([1, -4]))
        self.plot_and_assert(np.array([2, -3]))
        self.plot_and_assert(np.array([-2.2, -2]))
        self.plot_and_assert(np.array([-1.84, -2]))
        self.plot_and_assert(np.array([-2, -4]), on_path = False)

    def test_lines(self):
        self.plot_and_assert(np.array([-1.74, -2]))
        self.plot_and_assert(np.array([-1.74, 2]))
        self.plot_and_assert(np.array([0, 0]))
        self.plot_and_assert(np.array([0, 0]))
        self.plot_and_assert(np.array([1, 1]))
        self.plot_and_assert(np.array([2, 2]))
        self.plot_and_assert(np.array([-1, -1]))
        self.plot_and_assert(np.array([1.21, 1.21]))
        self.plot_and_assert(np.array([-0.32, -0.25]))
        self.plot_and_assert(np.array([0, -4]))
        self.plot_and_assert(np.array([0, -3.5]))
        self.plot_and_assert(np.array([0, -3]))
        self.plot_and_assert(np.array([0, 3]))
        self.plot_and_assert(np.array([0, 4.26]), on_path = False)
        self.plot_and_assert(np.array([0, -4.26]), on_path = False)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv.pop() == "-p":
        TestOnPath.PLOT_ON_TEARDOWN = True
    unittest.main()
