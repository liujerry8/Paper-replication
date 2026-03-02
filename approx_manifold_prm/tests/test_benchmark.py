#!/usr/bin/env python3
"""
test_benchmark.py

Unit tests for the benchmark components: TraditionalPRM and benchmark helpers.

Tests cover:
  - TraditionalPRM roadmap construction (unit-circle constraint)
  - TraditionalPRM query returns a path or None
  - Timer context manager records positive elapsed time
  - Benchmark single-trial runners produce expected dict keys
  - compute_stats aggregation produces correct success rate and timing stats
  - Path length helper
"""

import sys
import os
import unittest
import numpy as np

SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

from traditional_prm import TraditionalPRM
from benchmark import (
    Timer,
    run_am_prm_trial,
    run_traditional_prm_trial,
    compute_stats,
    _path_length,
    _circle_F,
    _circle_J,
    _CIRCLE_BOUNDS,
)


# ---------------------------------------------------------------------------
# Test constraint: unit circle (same as test_approx_manifold_graph.py)
# ---------------------------------------------------------------------------

def circle_F(x):
    return np.array([x[0] ** 2 + x[1] ** 2 - 1.0])


def circle_J(x):
    return np.array([[2 * x[0], 2 * x[1]]])


CIRCLE_BOUNDS = np.array([[-2.0, 2.0], [-2.0, 2.0]])

all_free = lambda x: True
no_obstacle_edge = lambda xa, xb: True


# ---------------------------------------------------------------------------
# Tests: TraditionalPRM
# ---------------------------------------------------------------------------

class TestTraditionalPRMBuild(unittest.TestCase):

    def test_roadmap_builds_with_samples(self):
        np.random.seed(42)
        planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
            constraint_fn=circle_F,
            jacobian_fn=circle_J,
        )
        roadmap, build_time = planner.build_roadmap(
            n_samples=30, max_time=10.0)
        self.assertGreater(roadmap.number_of_nodes(), 0)
        self.assertGreater(build_time, 0.0)

    def test_roadmap_has_edges(self):
        np.random.seed(42)
        planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
            constraint_fn=circle_F,
            jacobian_fn=circle_J,
        )
        roadmap, _ = planner.build_roadmap(n_samples=50, max_time=10.0)
        self.assertGreater(roadmap.number_of_edges(), 0)

    def test_roadmap_without_constraint(self):
        np.random.seed(42)
        planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
        )
        roadmap, _ = planner.build_roadmap(n_samples=20, max_time=5.0)
        self.assertEqual(roadmap.number_of_nodes(), 20)

    def test_build_returns_time(self):
        np.random.seed(1)
        planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
            constraint_fn=circle_F,
            jacobian_fn=circle_J,
        )
        _, build_time = planner.build_roadmap(n_samples=10, max_time=5.0)
        self.assertIsInstance(build_time, float)
        self.assertGreater(build_time, 0.0)


class TestTraditionalPRMQuery(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)
        self.planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
            constraint_fn=circle_F,
            jacobian_fn=circle_J,
        )
        self.planner.build_roadmap(n_samples=50, max_time=10.0)

    def test_query_returns_three_values(self):
        x_s = np.array([1.0, 0.0])
        x_g = np.array([0.0, 1.0])
        result = self.planner.query(x_s, x_g)
        self.assertEqual(len(result), 3)

    def test_query_planning_time_positive(self):
        x_s = np.array([1.0, 0.0])
        x_g = np.array([0.0, 1.0])
        _, _, t_ms = self.planner.query(x_s, x_g)
        self.assertGreater(t_ms, 0.0)

    def test_query_path_connects_endpoints(self):
        x_s = np.array([1.0, 0.0])
        x_g = np.array([0.0, 1.0])
        path, cost, _ = self.planner.query(x_s, x_g)
        if path is not None:
            np.testing.assert_allclose(path[0], x_s, atol=1e-6)
            np.testing.assert_allclose(path[-1], x_g, atol=1e-6)
            self.assertGreater(cost, 0)

    def test_empty_roadmap_query_returns_none(self):
        empty_planner = TraditionalPRM(
            config_dim=2,
            config_bounds=CIRCLE_BOUNDS,
            collision_checker=all_free,
        )
        path, cost, t_ms = empty_planner.query(
            np.array([1.0, 0.0]), np.array([0.0, 1.0]))
        self.assertIsNone(path)
        self.assertEqual(cost, float('inf'))


# ---------------------------------------------------------------------------
# Tests: Timer
# ---------------------------------------------------------------------------

class TestTimer(unittest.TestCase):

    def test_records_positive_time(self):
        with Timer() as t:
            _ = sum(range(1000))
        self.assertGreater(t.elapsed, 0.0)

    def test_elapsed_ms_conversion(self):
        with Timer() as t:
            _ = sum(range(1000))
        self.assertAlmostEqual(t.elapsed_ms, t.elapsed * 1000.0, places=6)


# ---------------------------------------------------------------------------
# Tests: Single-trial runners
# ---------------------------------------------------------------------------

class TestAMPRMTrial(unittest.TestCase):

    def test_returns_expected_keys(self):
        np.random.seed(42)
        result = run_am_prm_trial(
            constraint_fn=_circle_F,
            jacobian_fn=_circle_J,
            config_dim=2,
            manifold_dim=1,
            config_bounds=_CIRCLE_BOUNDS,
            collision_fn=all_free,
            edge_checker=no_obstacle_edge,
            x_start=np.array([1.0, 0.0]),
            x_goal=np.array([-1.0, 0.0]),
            n_c=20,
            n_e=2,
            roadmap_time=2.0,
            plan_time=5.0,
        )
        for key in ['graph_build_s', 'roadmap_build_s', 'plan_ms',
                     'success', 'path_length', 'offline_total_s']:
            self.assertIn(key, result)

    def test_timing_values_positive(self):
        np.random.seed(42)
        result = run_am_prm_trial(
            constraint_fn=_circle_F,
            jacobian_fn=_circle_J,
            config_dim=2,
            manifold_dim=1,
            config_bounds=_CIRCLE_BOUNDS,
            collision_fn=all_free,
            edge_checker=no_obstacle_edge,
            x_start=np.array([1.0, 0.0]),
            x_goal=np.array([-1.0, 0.0]),
            n_c=20,
            n_e=2,
            roadmap_time=2.0,
            plan_time=5.0,
        )
        self.assertGreater(result['graph_build_s'], 0.0)
        self.assertGreater(result['roadmap_build_s'], 0.0)
        self.assertGreater(result['plan_ms'], 0.0)


class TestTraditionalPRMTrial(unittest.TestCase):

    def test_returns_expected_keys(self):
        np.random.seed(42)
        result = run_traditional_prm_trial(
            config_dim=2,
            config_bounds=_CIRCLE_BOUNDS,
            collision_fn=all_free,
            constraint_fn=_circle_F,
            jacobian_fn=_circle_J,
            x_start=np.array([1.0, 0.0]),
            x_goal=np.array([-1.0, 0.0]),
            n_samples=30,
            max_time=5.0,
        )
        for key in ['roadmap_build_s', 'plan_ms', 'success',
                     'path_length', 'offline_total_s']:
            self.assertIn(key, result)

    def test_timing_values_positive(self):
        np.random.seed(42)
        result = run_traditional_prm_trial(
            config_dim=2,
            config_bounds=_CIRCLE_BOUNDS,
            collision_fn=all_free,
            constraint_fn=_circle_F,
            jacobian_fn=_circle_J,
            x_start=np.array([1.0, 0.0]),
            x_goal=np.array([-1.0, 0.0]),
            n_samples=30,
            max_time=5.0,
        )
        self.assertGreater(result['roadmap_build_s'], 0.0)
        self.assertGreater(result['plan_ms'], 0.0)


# ---------------------------------------------------------------------------
# Tests: compute_stats
# ---------------------------------------------------------------------------

class TestComputeStats(unittest.TestCase):

    def test_success_rate(self):
        trials = [
            {'success': True, 'offline_total_s': 1.0, 'plan_ms': 10.0,
             'path_length': 2.0, 'graph_build_s': 0.5, 'roadmap_build_s': 0.5},
            {'success': True, 'offline_total_s': 1.2, 'plan_ms': 12.0,
             'path_length': 2.1, 'graph_build_s': 0.6, 'roadmap_build_s': 0.6},
            {'success': False, 'offline_total_s': 1.1, 'plan_ms': 100.0,
             'path_length': None, 'graph_build_s': 0.55, 'roadmap_build_s': 0.55},
        ]
        stats = compute_stats(trials)
        self.assertAlmostEqual(stats['success_rate'], 2.0 / 3.0, places=4)

    def test_timing_stats_have_mean_std(self):
        trials = [
            {'success': True, 'offline_total_s': 1.0, 'plan_ms': 10.0,
             'path_length': 2.0, 'graph_build_s': 0.5, 'roadmap_build_s': 0.5},
            {'success': True, 'offline_total_s': 2.0, 'plan_ms': 20.0,
             'path_length': 3.0, 'graph_build_s': 1.0, 'roadmap_build_s': 1.0},
        ]
        stats = compute_stats(trials)
        self.assertIn('mean', stats['offline_time_s'])
        self.assertIn('std', stats['offline_time_s'])
        self.assertIn('mean', stats['plan_time_ms'])

    def test_empty_trials(self):
        stats = compute_stats([])
        self.assertEqual(stats, {})


# ---------------------------------------------------------------------------
# Tests: _path_length helper
# ---------------------------------------------------------------------------

class TestPathLength(unittest.TestCase):

    def test_straight_line(self):
        path = [np.array([0.0, 0.0]), np.array([3.0, 4.0])]
        self.assertAlmostEqual(_path_length(path), 5.0, places=6)

    def test_single_point(self):
        path = [np.array([1.0, 2.0])]
        self.assertAlmostEqual(_path_length(path), 0.0)

    def test_none_path(self):
        self.assertAlmostEqual(_path_length(None), 0.0)

    def test_empty_list_path(self):
        self.assertAlmostEqual(_path_length([]), 0.0)

    def test_multi_segment(self):
        path = [np.array([0.0]), np.array([1.0]), np.array([3.0])]
        self.assertAlmostEqual(_path_length(path), 3.0, places=6)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
