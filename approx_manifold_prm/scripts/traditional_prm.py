#!/usr/bin/env python3
"""
traditional_prm.py

Baseline Traditional PRM planner that samples directly in the full
configuration space (without the approximate manifold graph).

This serves as the control group for benchmarking against the proposed
AM-PRM method described in the paper.  The traditional PRM approach:

  1. Samples configurations uniformly in C-space.
  2. Optionally projects them onto the constraint manifold.
  3. Checks collision-free feasibility.
  4. Connects neighbours within a radius (or k-nearest) using a local planner.
  5. Answers queries via Dijkstra on the resulting roadmap.

By comparing construction time, online query time, success rate, and path
quality between this traditional PRM and the AM-PRM pipeline, we can
quantify the speedup introduced by the approximate manifold graph.
"""

import math
import time
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree


class TraditionalPRM:
    """
    Traditional PRM planner operating directly in configuration space.

    Parameters
    ----------
    config_dim : int
        Dimension of the configuration space.
    config_bounds : array-like of shape (n, 2)
        Lower and upper joint limits for each DOF.
    collision_checker : callable
        collision_checker(x) -> bool.  True if x is collision-free.
    constraint_fn : callable or None
        F(x) -> array.  If provided, sampled configurations are projected
        onto the constraint manifold via Newton-Raphson before acceptance.
    jacobian_fn : callable or None
        J(x) -> array.  Jacobian of constraint_fn (required when
        constraint_fn is set).
    constraint_tol : float
        Convergence tolerance for constraint projection.
    projection_max_iter : int
        Maximum Newton-Raphson iterations for projection.
    local_planner : callable or None
        local_planner(x_a, x_b) -> bool.  Returns True if the straight-line
        segment is collision-free.  If None a default interpolation checker
        is used.
    cost_fn : callable or None
        cost_fn(x_a, x_b) -> float.  Defaults to Euclidean distance.
    """

    def __init__(
        self,
        config_dim,
        config_bounds,
        collision_checker,
        constraint_fn=None,
        jacobian_fn=None,
        constraint_tol=1e-4,
        projection_max_iter=50,
        local_planner=None,
        cost_fn=None,
    ):
        self.n = config_dim
        self.bounds = np.array(config_bounds)
        self.is_free = collision_checker
        self.F = constraint_fn
        self.J = jacobian_fn
        self.proj_tol = constraint_tol
        self.proj_max_iter = projection_max_iter
        self.local_plan = local_planner or self._default_local_planner
        self.cost = cost_fn or (lambda a, b: float(np.linalg.norm(a - b)))

        self.roadmap = nx.Graph()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def build_roadmap(self, n_samples=500, k_neighbours=None, max_time=60.0):
        """
        Build a PRM roadmap by uniform sampling in C-space.

        Parameters
        ----------
        n_samples : int
            Target number of collision-free samples to add.
        k_neighbours : int or None
            Number of nearest neighbours to attempt connections.  If None,
            uses k = ceil(2e * log(n)) (PRM* formula).
        max_time : float
            Wall-clock time budget in seconds.

        Returns
        -------
        roadmap : nx.Graph
            Each node has attribute 'x' (configuration); each edge has
            attribute 'weight' (cost).
        build_time : float
            Total construction time in seconds.
        """
        self.roadmap = nx.Graph()
        t_start = time.time()
        added = 0
        attempts = 0
        max_attempts = n_samples * 200

        # --- Phase 1: sample vertices ---
        while added < n_samples and attempts < max_attempts:
            if time.time() - t_start >= max_time:
                break
            attempts += 1
            x = self._random_config()

            # Project onto constraint manifold if a constraint is specified
            if self.F is not None:
                x = self._project(x)
                if x is None:
                    continue

            if not self.is_free(x):
                continue

            node_id = added
            self.roadmap.add_node(node_id, x=x)
            added += 1

        # --- Phase 2: connect edges ---
        if self.roadmap.number_of_nodes() >= 2:
            nodes = list(self.roadmap.nodes(data=True))
            configs = np.array([d['x'] for _, d in nodes])
            ids = [nid for nid, _ in nodes]
            tree = cKDTree(configs)

            for i, (nid, data) in enumerate(nodes):
                if time.time() - t_start >= max_time:
                    break
                k = k_neighbours
                if k is None:
                    n_total = self.roadmap.number_of_nodes()
                    k = max(1, math.ceil(2 * math.e * math.log(max(2, n_total))))
                k_query = min(k + 1, len(configs))
                dists, idxs = tree.query(data['x'], k=k_query)
                for dist, j in zip(dists, idxs):
                    nb_id = ids[j]
                    if nb_id == nid:
                        continue
                    if self.roadmap.has_edge(nid, nb_id):
                        continue
                    nb_x = configs[j]
                    if self.local_plan(data['x'], nb_x):
                        c = self.cost(data['x'], nb_x)
                        self.roadmap.add_edge(nid, nb_id, weight=c)

        build_time = time.time() - t_start
        return self.roadmap, build_time

    def query(self, x_start, x_goal, connect_k=10):
        """
        Find the shortest path in the roadmap from x_start to x_goal.

        Parameters
        ----------
        x_start, x_goal : np.ndarray
        connect_k : int
            Number of nearest roadmap nodes to try when connecting endpoints.

        Returns
        -------
        path : list of np.ndarray or None
        cost : float
        planning_time_ms : float
            Wall-clock query time in milliseconds.
        """
        t0 = time.time()
        if self.roadmap.number_of_nodes() == 0:
            return None, float('inf'), (time.time() - t0) * 1000

        START_ID, GOAL_ID = -1, -2
        self.roadmap.add_node(START_ID, x=x_start)
        self.roadmap.add_node(GOAL_ID, x=x_goal)

        for node_id, x in [(START_ID, x_start), (GOAL_ID, x_goal)]:
            neighbours = self._k_nearest(x, connect_k, exclude={START_ID, GOAL_ID})
            for nb_id, nb_x in neighbours:
                if self.local_plan(x, nb_x):
                    self.roadmap.add_edge(node_id, nb_id,
                                          weight=self.cost(x, nb_x))

        path = None
        total_cost = float('inf')
        try:
            node_path = nx.dijkstra_path(
                self.roadmap, START_ID, GOAL_ID, weight='weight')
            total_cost = nx.dijkstra_path_length(
                self.roadmap, START_ID, GOAL_ID, weight='weight')
            path = [self.roadmap.nodes[nid]['x'] for nid in node_path]
        except nx.NetworkXNoPath:
            pass

        self.roadmap.remove_nodes_from([START_ID, GOAL_ID])
        t_ms = (time.time() - t0) * 1000
        return path, total_cost, t_ms

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _random_config(self):
        lo, hi = self.bounds[:, 0], self.bounds[:, 1]
        return lo + np.random.rand(self.n) * (hi - lo)

    def _project(self, x0):
        """Newton-Raphson projection onto the constraint manifold."""
        x = np.clip(x0.copy(), self.bounds[:, 0], self.bounds[:, 1])
        for _ in range(self.proj_max_iter):
            f_val = self.F(x)
            if np.linalg.norm(f_val) < self.proj_tol:
                return x
            J_val = self.J(x)
            J_pinv = np.linalg.pinv(J_val)
            x = x - J_pinv @ f_val
            x = np.clip(x, self.bounds[:, 0], self.bounds[:, 1])
        if np.linalg.norm(self.F(x)) < self.proj_tol:
            return x
        return None

    def _k_nearest(self, x, k, exclude=None):
        exclude = set(exclude or [])
        candidates = [
            (nid, data['x'])
            for nid, data in self.roadmap.nodes(data=True)
            if nid not in exclude
        ]
        if not candidates:
            return []
        ids, configs = zip(*candidates)
        configs_arr = np.array(configs)
        dists = np.linalg.norm(configs_arr - x, axis=1)
        k = min(k, len(dists))
        nearest_idx = np.argpartition(dists, k - 1)[:k]
        return [(ids[i], configs[i]) for i in nearest_idx]

    def _default_local_planner(self, x_a, x_b, n_steps=10):
        for t in np.linspace(0, 1, n_steps + 2)[1:-1]:
            x = (1 - t) * x_a + t * x_b
            if not self.is_free(x):
                return False
        return True
