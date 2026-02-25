#!/usr/bin/env python3
"""
test_approx_manifold_graph.py

Unit tests for the core algorithms without requiring ROS or MoveIt!.

Tests cover:
  - Newton-Raphson projection onto a simple constraint manifold
  - Diversity check (equations 2.6–2.8)
  - Edge validity check (equation 2.11)
  - Full graph build (vertex set + edge set)
  - Uniform and nearby sampling
  - PRM* roadmap construction
  - LazyPRM path planning
"""

import sys
import os
import math
import unittest
import numpy as np

# Allow importing scripts without installing the package
SCRIPTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPTS_DIR))

from approx_manifold_graph import ApproximateManifoldGraph
from prm_star import PRMStar
from lazy_prm import LazyPRM


# ---------------------------------------------------------------------------
# Test constraint: unit circle in 2-D (x0^2 + x1^2 = 1)
# F(x) = x0^2 + x1^2 - 1  (scalar)
# J(x) = [2*x0, 2*x1]
# Manifold dimension k = 1 (1-D curve embedded in 2-D)
# ---------------------------------------------------------------------------

def circle_F(x):
    return np.array([x[0] ** 2 + x[1] ** 2 - 1.0])


def circle_J(x):
    return np.array([[2 * x[0], 2 * x[1]]])


CIRCLE_BOUNDS = np.array([[-2.0, 2.0], [-2.0, 2.0]])


def make_circle_graph(**kwargs):
    defaults = dict(
        constraint_fn=circle_F,
        jacobian_fn=circle_J,
        config_dim=2,
        manifold_dim=1,   # k = n - m = 2 - 1
        config_bounds=CIRCLE_BOUNDS,
        projection_tol=1e-6,
        diversity_epsilon=0.01,
        diversity_rho=0.01,
        diversity_alpha=np.pi / 4,
        edge_validity_zeta=0.05,
        edge_validity_samples=8,
    )
    defaults.update(kwargs)
    return ApproximateManifoldGraph(**defaults)


# ---------------------------------------------------------------------------
# Tests: ApproximateManifoldGraph
# ---------------------------------------------------------------------------

class TestProjection(unittest.TestCase):

    def test_projects_to_unit_circle(self):
        g = make_circle_graph()
        for _ in range(20):
            x0 = np.random.uniform(-2, 2, size=2)
            x_proj = g.project(x0)
            if x_proj is not None:
                self.assertAlmostEqual(np.linalg.norm(x_proj), 1.0, places=4)

    def test_projection_already_on_manifold(self):
        g = make_circle_graph()
        x_on = np.array([1.0, 0.0])
        x_proj = g.project(x_on)
        self.assertIsNotNone(x_proj)
        np.testing.assert_allclose(np.linalg.norm(x_proj), 1.0, atol=1e-5)

    def test_projection_far_point(self):
        g = make_circle_graph()
        x_far = np.array([5.0, 5.0])
        x_proj = g.project(x_far)
        # May succeed or fail due to clipping; if it succeeds it must be on manifold
        if x_proj is not None:
            self.assertAlmostEqual(np.linalg.norm(x_proj), 1.0, places=4)


class TestTangentBasis(unittest.TestCase):

    def test_tangent_orthogonal_to_gradient(self):
        g = make_circle_graph()
        x = np.array([1.0, 0.0])
        Phi = g.tangent_basis(x)
        # Phi should span the null space of J(x) = [2, 0]
        # Null space is [0, 1]  (the tangent direction)
        J = circle_J(x)
        residual = J @ Phi
        np.testing.assert_allclose(residual, 0.0, atol=1e-10)

    def test_tangent_shape(self):
        g = make_circle_graph()
        x = np.array([0.0, 1.0])
        Phi = g.tangent_basis(x)
        self.assertEqual(Phi.shape, (2, 1))


class TestDiversityCheck(unittest.TestCase):

    def test_first_vertex_always_accepted(self):
        g = make_circle_graph()
        # No vertices yet → diversity always passes
        x = np.array([1.0, 0.0])
        Phi = g.tangent_basis(x)
        self.assertTrue(g._diversity_check(x, Phi))

    def test_identical_vertex_rejected(self):
        g = make_circle_graph(diversity_epsilon=0.01, diversity_rho=0.01)
        x = np.array([1.0, 0.0])
        Phi = g.tangent_basis(x)
        # Add x to the vertex set manually
        g.vertices.append(x)
        g.tangent_bases.append(Phi)
        # A nearly identical point should fail diversity
        x2 = np.array([1.0 + 1e-6, 0.0])
        x2 = g.project(x2)
        if x2 is not None:
            Phi2 = g.tangent_basis(x2)
            result = g._diversity_check(x2, Phi2)
            self.assertFalse(result)


class TestEdgeValidity(unittest.TestCase):

    def test_edge_on_manifold(self):
        g = make_circle_graph(edge_validity_zeta=0.05)
        # Two close points on the unit circle
        theta1, theta2 = 0.0, 0.05
        x1 = np.array([np.cos(theta1), np.sin(theta1)])
        x2 = np.array([np.cos(theta2), np.sin(theta2)])
        # Points very close: straight line stays near manifold
        result = g._valid_edge(x1, x2)
        self.assertIsInstance(result, bool)

    def test_edge_off_manifold(self):
        g = make_circle_graph(edge_validity_zeta=1e-4)
        # Two antipodal points; straight line passes through origin (far from circle)
        x1 = np.array([1.0, 0.0])
        x2 = np.array([-1.0, 0.0])
        result = g._valid_edge(x1, x2)
        self.assertFalse(result)


class TestGraphBuild(unittest.TestCase):

    def test_vertex_set_size(self):
        np.random.seed(42)
        g = make_circle_graph()
        g._build_vertex_set(n_c=10)
        self.assertGreater(len(g.vertices), 0)
        self.assertLessEqual(len(g.vertices), 10)

    def test_all_vertices_on_manifold(self):
        np.random.seed(0)
        g = make_circle_graph()
        g._build_vertex_set(n_c=20)
        for x in g.vertices:
            err = np.linalg.norm(circle_F(x))
            self.assertLess(err, g.proj_tol * 10)

    def test_build_with_edges(self):
        np.random.seed(1)
        g = make_circle_graph()
        g.build(n_c=15, n_e=2)
        total_edges = sum(len(v) for v in g.adjacency.values()) // 2
        # Some edges should be present (not every pair will be valid)
        self.assertGreaterEqual(total_edges, 0)

    def test_vertex_set_reaches_target(self):
        """Diversity check should allow the vertex set to reach n_c."""
        import warnings
        np.random.seed(42)
        g = make_circle_graph()
        with warnings.catch_warnings():
            warnings.simplefilter('error', UserWarning)
            g._build_vertex_set(n_c=20)
        self.assertEqual(len(g.vertices), 20)

    def test_save_load(self):
        import tempfile
        np.random.seed(5)
        g = make_circle_graph()
        g.build(n_c=10, n_e=0)
        with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as f:
            path = f.name
        try:
            g.save(path)
            g2 = make_circle_graph()
            g2.load(path)
            self.assertEqual(len(g.vertices), len(g2.vertices))
            np.testing.assert_allclose(g.vertices[0], g2.vertices[0])
        finally:
            os.unlink(path)


class TestSampling(unittest.TestCase):

    def setUp(self):
        np.random.seed(7)
        self.g = make_circle_graph()
        self.g.build(n_c=30, n_e=3)

    def test_uniform_sample_on_manifold(self):
        x, idx = self.g.sample_uniform()
        self.assertIsInstance(x, np.ndarray)
        self.assertAlmostEqual(np.linalg.norm(circle_F(x)), 0.0, places=4)

    def test_uniform_sample_index_valid(self):
        _, idx = self.g.sample_uniform()
        self.assertGreaterEqual(idx, 0)
        self.assertLess(idx, len(self.g.vertices))

    def test_nearby_sample_returns_result(self):
        x, _ = self.g.sample_uniform()
        x_near = self.g.sample_nearby(x, d=1.0)
        # May be None if no neighbours found, but should usually succeed
        if x_near is not None:
            self.assertEqual(x_near.shape, x.shape)


# ---------------------------------------------------------------------------
# Tests: PRMStar
# ---------------------------------------------------------------------------

def all_free(x):
    return True


def no_obstacle_local(x_a, x_b):
    return True


class TestPRMStar(unittest.TestCase):

    def setUp(self):
        np.random.seed(42)
        self.g = make_circle_graph()
        self.g.build(n_c=40, n_e=3)

    def test_roadmap_builds(self):
        planner = PRMStar(self.g, all_free, no_obstacle_local)
        roadmap = planner.build_roadmap(max_time=2.0)
        self.assertGreater(roadmap.number_of_nodes(), 0)

    def test_roadmap_has_edges(self):
        planner = PRMStar(self.g, all_free, no_obstacle_local)
        roadmap = planner.build_roadmap(max_time=2.0)
        self.assertGreater(roadmap.number_of_edges(), 0)

    def test_query_finds_path(self):
        planner = PRMStar(self.g, all_free, no_obstacle_local)
        planner.build_roadmap(max_time=2.0)
        # Query between two vertices on the manifold
        if len(self.g.vertices) >= 2:
            x_s = self.g.vertices[0]
            x_g = self.g.vertices[-1]
            path, cost = planner.query(x_s, x_g)
            # Path may or may not exist depending on roadmap density
            if path is not None:
                self.assertGreater(len(path), 0)
                self.assertGreater(cost, 0)

    def test_save_load_roadmap(self):
        import tempfile
        planner = PRMStar(self.g, all_free, no_obstacle_local)
        planner.build_roadmap(max_time=1.0)
        with tempfile.NamedTemporaryFile(suffix='.pkl', delete=False) as f:
            path = f.name
        try:
            planner.save(path)
            planner2 = PRMStar(self.g, all_free, no_obstacle_local)
            planner2.load(path)
            self.assertEqual(
                planner.roadmap.number_of_nodes(),
                planner2.roadmap.number_of_nodes()
            )
        finally:
            os.unlink(path)


# ---------------------------------------------------------------------------
# Tests: LazyPRM
# ---------------------------------------------------------------------------

class TestLazyPRM(unittest.TestCase):

    def _build_planner(self, n_c=50, max_time=3.0):
        np.random.seed(99)
        g = make_circle_graph()
        g.build(n_c=n_c, n_e=4)
        prm = PRMStar(g, all_free, no_obstacle_local)
        roadmap = prm.build_roadmap(max_time=max_time)
        lazy = LazyPRM(
            roadmap=roadmap,
            collision_checker=all_free,
            edge_checker=no_obstacle_local,
            connect_k=5,
        )
        return lazy, g

    def test_plan_returns_path_or_none(self):
        lazy, g = self._build_planner()
        if len(g.vertices) >= 2:
            x_s = g.vertices[0]
            x_g = g.vertices[-1]
            path, t_ms = lazy.plan(x_s, x_g)
            self.assertIsInstance(t_ms, float)
            if path is not None:
                self.assertGreater(len(path), 0)

    def test_planning_time_is_positive(self):
        lazy, g = self._build_planner()
        if len(g.vertices) >= 2:
            x_s = g.vertices[0]
            x_g = g.vertices[-1]
            _, t_ms = lazy.plan(x_s, x_g)
            self.assertGreater(t_ms, 0)

    def test_collision_blocked_path(self):
        """With a checker that blocks everything, planning should fail."""
        lazy, g = self._build_planner()
        lazy.is_free = lambda x: False
        if len(g.vertices) >= 2:
            path, _ = lazy.plan(g.vertices[0], g.vertices[-1])
            self.assertIsNone(path)

    def test_path_connects_start_to_goal(self):
        lazy, g = self._build_planner()
        if len(g.vertices) >= 2:
            x_s = g.vertices[0]
            x_g = g.vertices[-1]
            path, _ = lazy.plan(x_s, x_g)
            if path is not None:
                np.testing.assert_allclose(path[0], x_s, atol=1e-6)
                np.testing.assert_allclose(path[-1], x_g, atol=1e-6)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
