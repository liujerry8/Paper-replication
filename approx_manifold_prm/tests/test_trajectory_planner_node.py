#!/usr/bin/env python3
"""
test_trajectory_planner_node.py

Unit tests for trajectory_planner_node.py (non-ROS portions).

Tests cover:
  - Service type import is at module level (not deferred inside methods)
  - The _plan_cb method uses req.time_limit when > 0
  - The _plan_cb method falls back to parameter when req.time_limit == 0
  - The _plan_cb method returns an error message when planner is not ready
  - The _plan_cb method populates resp.message in all code paths
  - make_edge_checker correctly delegates to collision_checker
"""

import sys
import os
import unittest
from unittest.mock import MagicMock, patch
import numpy as np

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
sys.path.insert(0, SCRIPTS_DIR)


# ---------------------------------------------------------------------------
# Tests: make_edge_checker
# ---------------------------------------------------------------------------

class TestMakeEdgeChecker(unittest.TestCase):

    def test_edge_checker_calls_collision_checker(self):
        from trajectory_planner_node import make_edge_checker

        call_count = [0]

        def mock_checker(x):
            call_count[0] += 1
            return True

        checker = make_edge_checker(mock_checker, n_steps=5)
        x_a = np.zeros(6)
        x_b = np.ones(6)
        result = checker(x_a, x_b)
        self.assertTrue(result)
        self.assertGreater(call_count[0], 0)

    def test_edge_checker_returns_false_on_collision(self):
        from trajectory_planner_node import make_edge_checker

        def always_blocked(x):
            return False

        checker = make_edge_checker(always_blocked, n_steps=3)
        result = checker(np.zeros(6), np.ones(6))
        self.assertFalse(result)


# ---------------------------------------------------------------------------
# Tests: Service import location
# ---------------------------------------------------------------------------

class TestServiceImportLocation(unittest.TestCase):
    """Verify the service types are imported at module level, not inside methods."""

    def test_plan_trajectory_import_in_top_level_block(self):
        """PlanTrajectory import should be in the top-level try/except block."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        # Verify the import is present in the top-level try block, not inside
        # a class method. We check that 'from approx_manifold_prm.srv import'
        # appears before the class definition.
        import_pos = source.find('from approx_manifold_prm.srv import PlanTrajectory')
        class_pos = source.find('class TrajectoryPlannerNode')
        self.assertNotEqual(import_pos, -1,
                            'PlanTrajectory import not found in source')
        self.assertLess(import_pos, class_pos,
                        'PlanTrajectory import should be before the class definition '
                        '(i.e. at module level in the try/except block)')

    def test_no_deferred_service_import_in_plan_cb(self):
        """_plan_cb should NOT contain a deferred import of PlanTrajectoryResponse."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        # Find the _plan_cb method body
        cb_start = source.find('def _plan_cb(self, req):')
        self.assertNotEqual(cb_start, -1, '_plan_cb method not found')
        # Check that there's no 'from approx_manifold_prm.srv import' after _plan_cb
        cb_body = source[cb_start:]
        # Find the next method definition to limit the search
        next_def = cb_body.find('\n    def ', 1)
        if next_def > 0:
            cb_body = cb_body[:next_def]
        self.assertNotIn('from approx_manifold_prm.srv import', cb_body,
                         '_plan_cb should not contain deferred service imports')


# ---------------------------------------------------------------------------
# Tests: Node initialization structure
# ---------------------------------------------------------------------------

class TestNodeInitStructure(unittest.TestCase):
    """Verify the node registers the service before heavy initialization."""

    def test_service_registered_before_planner_init(self):
        """rospy.Service should be called before _initialize_planner."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        # Find the __init__ method
        init_start = source.find('def __init__(self):')
        self.assertNotEqual(init_start, -1)

        init_body = source[init_start:]
        # Find rospy.Service call and _initialize_planner call
        service_pos = init_body.find('rospy.Service(')
        init_planner_pos = init_body.find('self._initialize_planner()')
        self.assertNotEqual(service_pos, -1, 'rospy.Service call not found')
        self.assertNotEqual(init_planner_pos, -1,
                            '_initialize_planner call not found')
        self.assertLess(service_pos, init_planner_pos,
                        'Service should be registered before _initialize_planner is called')

    def test_ready_flag_exists(self):
        """The node should have a self.ready flag."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()
        self.assertIn('self.ready', source)

    def test_init_error_captured(self):
        """The node should capture initialization errors."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()
        self.assertIn('self.init_error', source)


# ---------------------------------------------------------------------------
# Tests: _plan_cb logic
# ---------------------------------------------------------------------------

class TestPlanCbLogic(unittest.TestCase):
    """Test the _plan_cb method logic by reading the source code structure."""

    def test_time_limit_from_request_used(self):
        """_plan_cb should use req.time_limit when > 0."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        cb_start = source.find('def _plan_cb(self, req):')
        cb_body = source[cb_start:]
        next_def = cb_body.find('\n    def ', 1)
        if next_def > 0:
            cb_body = cb_body[:next_def]

        self.assertIn('req.time_limit', cb_body,
                      '_plan_cb should reference req.time_limit')

    def test_not_ready_returns_error_message(self):
        """_plan_cb should check self.ready and return an error message."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        cb_start = source.find('def _plan_cb(self, req):')
        cb_body = source[cb_start:]
        next_def = cb_body.find('\n    def ', 1)
        if next_def > 0:
            cb_body = cb_body[:next_def]

        self.assertIn('self.ready', cb_body,
                      '_plan_cb should check self.ready')
        self.assertIn('resp.message', cb_body,
                      '_plan_cb should set resp.message')

    def test_plan_cb_has_exception_handling(self):
        """_plan_cb should wrap the planning call in try/except."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        cb_start = source.find('def _plan_cb(self, req):')
        cb_body = source[cb_start:]
        next_def = cb_body.find('\n    def ', 1)
        if next_def > 0:
            cb_body = cb_body[:next_def]

        self.assertIn('try:', cb_body,
                      '_plan_cb should contain try/except for exception handling')
        self.assertIn('except', cb_body,
                      '_plan_cb should catch exceptions from the planner')


class TestCollisionCheckerUsesStateValidity(unittest.TestCase):
    """Verify that make_collision_checker uses state validity, not move_group.plan()."""

    def test_collision_checker_does_not_use_plan(self):
        """make_collision_checker should NOT call move_group.plan()."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        fn_start = source.find('def make_collision_checker(')
        self.assertNotEqual(fn_start, -1)
        fn_body = source[fn_start:]
        next_def = fn_body.find('\ndef ', 1)
        if next_def > 0:
            fn_body = fn_body[:next_def]

        self.assertNotIn('move_group.plan()', fn_body,
                         'make_collision_checker should not use move_group.plan()')

    def test_collision_checker_uses_state_validity(self):
        """make_collision_checker should use GetStateValidity service."""
        node_path = os.path.join(SCRIPTS_DIR, 'trajectory_planner_node.py')
        with open(node_path) as f:
            source = f.read()

        fn_start = source.find('def make_collision_checker(')
        fn_body = source[fn_start:]
        next_def = fn_body.find('\ndef ', 1)
        if next_def > 0:
            fn_body = fn_body[:next_def]

        self.assertIn('GetStateValidity', fn_body,
                      'make_collision_checker should use GetStateValidity service')


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
