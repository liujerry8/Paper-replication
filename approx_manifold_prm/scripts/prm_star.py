#!/usr/bin/env python3
"""
prm_star.py

Implements the PRM* algorithm on an Approximate Manifold Graph (Algorithm 2.3).

PRM* [Karaman & Frazzoli, 2011] is an asymptotically optimal probabilistic
roadmap planner.  Here it runs on the pre-built approximate manifold graph G
to produce an offline roadmap M that contains a near-optimal trajectory between
the start and goal configurations.

Key idea (Section 2.2.4):
  - Sample collision-free vertices from G.
  - Connect each vertex to its k nearest neighbours in M, where k = 2e*log(n).
  - Run until the cost converges or the time limit is reached.
  - The resulting roadmap M is stored for fast online re-use by LazyPRM.
"""

import math
import time
import pickle
import numpy as np
import networkx as nx
from scipy.spatial import cKDTree


class PRMStar:
    """
    Offline PRM* planner operating on an ApproximateManifoldGraph.

    Parameters
    ----------
    approx_graph : ApproximateManifoldGraph
        Pre-built approximate manifold graph G = (V, E).
    collision_checker : callable
        collision_checker(x) -> bool.  Returns True if x is collision-free.
    local_planner : callable, optional
        local_planner(x_a, x_b) -> bool.  Returns True if the straight-line
        segment from x_a to x_b is collision-free.  If None, a default
        straight-line checker using collision_checker is used.
    cost_fn : callable, optional
        cost_fn(x_a, x_b) -> float.  Edge cost (defaults to Euclidean distance).
    convergence_window : int
        Number of consecutive best-costs to compare for convergence (default 5).
    convergence_threshold : float
        Stop when the relative change in average best-cost falls below this
        value (default 0.10, i.e. 10%).
    """

    def __init__(
        self,
        approx_graph,
        collision_checker,
        local_planner=None,
        cost_fn=None,
        convergence_window=5,
        convergence_threshold=0.10,
    ):
        self.G = approx_graph
        self.is_free = collision_checker
        self.local_plan = local_planner or self._default_local_planner
        self.cost = cost_fn or (lambda a, b: float(np.linalg.norm(a - b)))
        self.conv_window = convergence_window
        self.conv_thresh = convergence_threshold

        self.roadmap = nx.Graph()          # The offline roadmap M
        self._recent_best_costs = []       # Track convergence

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def build_roadmap(self, max_time=60.0, max_iter=None):
        """
        Generate the offline roadmap M (Algorithm 2.3).

        Parameters
        ----------
        max_time : float
            Maximum planning time in seconds.
        max_iter : int | None
            Maximum iterations (overrides convergence check when set).

        Returns
        -------
        roadmap : nx.Graph
            The offline roadmap; each node stores the configuration 'x'.
        """
        self.roadmap = nx.Graph()
        k_prm = 2 * math.e          # k_PRM* base constant
        n = 0
        t_start = time.time()

        while True:
            n += 1
            elapsed = time.time() - t_start
            if elapsed >= max_time:
                break
            if max_iter is not None and n > max_iter:
                break

            # Sample a configuration from the approximate graph
            v, _ = self.G.sample_uniform()

            # Skip if in collision
            if not self.is_free(v):
                continue

            # Determine k = ceil(k_PRM* * log(n))
            k = max(1, math.ceil(k_prm * math.log(n)))

            # Find k nearest neighbours in the current roadmap
            neighbours = self._k_nearest(v, k)

            # Add vertex to roadmap
            node_id = n
            self.roadmap.add_node(node_id, x=v)

            # Try to connect to each neighbour
            for nb_id, nb_x in neighbours:
                if self.local_plan(v, nb_x):
                    c = self.cost(v, nb_x)
                    self.roadmap.add_edge(node_id, nb_id, weight=c)

            # Check convergence (track best path costs for a fixed pair if possible)
            if self._check_convergence():
                break

        return self.roadmap

    def query(self, x_start, x_goal):
        """
        Find the shortest path in the roadmap between x_start and x_goal.

        Adds temporary start/goal nodes, runs Dijkstra, then removes them.

        Parameters
        ----------
        x_start, x_goal : np.ndarray

        Returns
        -------
        path : list of np.ndarray | None
            Ordered list of configurations from start to goal, or None if no
            path was found.
        cost : float
            Total path cost, or inf if no path.
        """
        if self.roadmap.number_of_nodes() == 0:
            return None, float('inf')

        START_ID, GOAL_ID = -1, -2
        self.roadmap.add_node(START_ID, x=x_start)
        self.roadmap.add_node(GOAL_ID, x=x_goal)

        # Connect start/goal to k nearest nodes in the roadmap
        k = max(1, math.ceil(2 * math.e * math.log(max(2, self.roadmap.number_of_nodes()))))
        for node_id, src_x, query_x in [
            (START_ID, x_start, x_start),
            (GOAL_ID, x_goal, x_goal),
        ]:
            neighbours = self._k_nearest(query_x, k, exclude=[START_ID, GOAL_ID])
            for nb_id, nb_x in neighbours:
                if self.local_plan(query_x, nb_x):
                    self.roadmap.add_edge(node_id, nb_id, weight=self.cost(query_x, nb_x))

        path = None
        total_cost = float('inf')
        try:
            node_path = nx.dijkstra_path(self.roadmap, START_ID, GOAL_ID, weight='weight')
            total_cost = nx.dijkstra_path_length(self.roadmap, START_ID, GOAL_ID, weight='weight')
            path = [self.roadmap.nodes[nid]['x'] for nid in node_path]
        except nx.NetworkXNoPath:
            pass

        self.roadmap.remove_nodes_from([START_ID, GOAL_ID])
        return path, total_cost

    def save(self, path):
        """Serialize the roadmap to a file."""
        with open(path, 'wb') as f:
            pickle.dump(self.roadmap, f)

    def load(self, path):
        """Load a previously serialized roadmap."""
        with open(path, 'rb') as f:
            self.roadmap = pickle.load(f)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _k_nearest(self, x, k, exclude=None):
        """
        Return the k nearest nodes in the current roadmap to configuration x.

        Returns a list of (node_id, configuration) tuples.
        """
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

    def _check_convergence(self):
        """
        Return True when the average best-cost has converged.

        Convergence criterion (Section 2.3): the relative change in average
        cost of the top-5 recent samples falls below convergence_threshold.
        """
        # Collect all edge costs as a proxy for best-path quality
        if self.roadmap.number_of_edges() == 0:
            return False
        total = sum(d.get('weight', 1.0) for _, _, d in self.roadmap.edges(data=True))
        self._recent_best_costs.append(total)

        if len(self._recent_best_costs) > self.conv_window * 2:
            self._recent_best_costs.pop(0)

        if len(self._recent_best_costs) < self.conv_window * 2:
            return False

        prev_avg = np.mean(self._recent_best_costs[:self.conv_window])
        curr_avg = np.mean(self._recent_best_costs[self.conv_window:])
        if prev_avg == 0:
            return False
        return abs(curr_avg - prev_avg) / prev_avg < self.conv_thresh

    def _default_local_planner(self, x_a, x_b, n_steps=10):
        """
        Straight-line local planner: check n_steps interpolated configurations.
        """
        for t in np.linspace(0, 1, n_steps + 2)[1:-1]:
            x = (1 - t) * x_a + t * x_b
            if not self.is_free(x):
                return False
        return True
