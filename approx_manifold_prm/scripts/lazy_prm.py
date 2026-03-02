#!/usr/bin/env python3
"""
lazy_prm.py

Implements the LazyPRM online trajectory planner described in Section 2.2.5.

LazyPRM [Bohlin & Kavraki, 2000] speeds up query time by deferring collision
checks until a path is found in the roadmap.  Only the edges that lie on the
returned path are collision-checked.  Edges that fail are removed, and the
search is repeated.  This is ideal when:

  * The roadmap M was built offline (expensive, done once).
  * The online scene may differ slightly from the offline scene (e.g. mobile
    robot re-positioning errors, objects moved by lab personnel).
  * Fast per-query planning is required (< 300 ms target from the paper).

Usage in the paper (Section 2.2.5 + 2.3.2):
  1. Load pre-built offline roadmap M from disk.
  2. On each new task, call plan(x_start, x_goal).
  3. Post-process the resulting joint-space path with MoveIt!'s IPTP
     (Iterative Parabolic Time Parameterization) to get a time-parameterized
     trajectory with velocity and acceleration profiles.
"""

import copy
import time
import numpy as np
import networkx as nx


class LazyPRM:
    """
    LazyPRM online planner.

    Parameters
    ----------
    roadmap : nx.Graph
        Offline roadmap M built by PRMStar.build_roadmap().  Each node must
        have a 'x' attribute (np.ndarray configuration) and each edge a
        'weight' attribute (cost).  Collision validity flags are stored as
        edge attribute 'valid' (None = unknown, True = free, False = blocked).
    collision_checker : callable
        collision_checker(x) -> bool.  Returns True if configuration x is
        collision-free in the *current* scene.
    edge_checker : callable
        edge_checker(x_a, x_b) -> bool.  Returns True if the straight-line
        segment between x_a and x_b is collision-free in the current scene.
    connect_k : int
        Number of nearest roadmap nodes to try when connecting start/goal.
    max_iterations : int
        Maximum re-plan iterations before giving up.
    """

    def __init__(
        self,
        roadmap,
        collision_checker,
        edge_checker,
        connect_k=10,
        max_iterations=100,
    ):
        self.roadmap = roadmap.copy()     # working copy (we mark edges invalid)
        self.is_free = collision_checker
        self.edge_free = edge_checker
        self.connect_k = connect_k
        self.max_iter = max_iterations

        # Initialise all edge validity to unknown (None)
        for u, v in self.roadmap.edges():
            self.roadmap[u][v]['valid'] = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def plan(self, x_start, x_goal, time_limit=10.0):
        """
        Find a collision-free path from x_start to x_goal using LazyPRM.

        Parameters
        ----------
        x_start, x_goal : np.ndarray
            Start and goal configurations (must be on the constraint manifold).
        time_limit : float
            Maximum planning time in seconds.

        Returns
        -------
        path : list of np.ndarray | None
            Sequence of collision-free configurations, or None on failure.
        planning_time_ms : float
            Wall-clock planning time in milliseconds.
        """
        t0 = time.time()
        START, GOAL = 'start', 'goal'
        working = copy.deepcopy(self.roadmap)

        # Connect start and goal to the roadmap
        if not self._connect_endpoints(working, START, x_start):
            return None, (time.time() - t0) * 1000
        if not self._connect_endpoints(working, GOAL, x_goal):
            working.remove_node(START)
            return None, (time.time() - t0) * 1000

        for iteration in range(self.max_iter):
            if time.time() - t0 > time_limit:
                break

            # Graph search (ignoring unknown validity — lazy strategy)
            try:
                node_path = nx.dijkstra_path(working, START, GOAL, weight='weight')
            except (nx.NetworkXNoPath, nx.NodeNotFound):
                break

            # Validate edges along the found path
            valid = True
            for u, v in zip(node_path[:-1], node_path[1:]):
                x_u = working.nodes[u]['x']
                x_v = working.nodes[v]['x']

                edge_valid = working[u][v].get('valid')
                if edge_valid is None:
                    # Lazy: check now
                    edge_valid = (
                        self.is_free(x_u)
                        and self.is_free(x_v)
                        and self.edge_free(x_u, x_v)
                    )
                    working[u][v]['valid'] = edge_valid

                if not edge_valid:
                    working.remove_edge(u, v)
                    valid = False
                    break   # Re-run graph search with the blocked edge removed

            if valid:
                path = [working.nodes[nid]['x'] for nid in node_path]
                return path, (time.time() - t0) * 1000

        return None, (time.time() - t0) * 1000

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _connect_endpoints(self, graph, node_id, x, max_attempts=None):
        """
        Add node_id with configuration x to graph and connect it to the
        connect_k nearest existing nodes that are reachable via a free edge.

        Returns True if at least one connection was made.
        """
        if not self.is_free(x):
            return False

        graph.add_node(node_id, x=x)
        neighbours = self._k_nearest(graph, x, self.connect_k, exclude={node_id})
        connected = False
        for nb_id, nb_x in neighbours:
            if self.edge_free(x, nb_x):
                c = float(np.linalg.norm(x - nb_x))
                graph.add_edge(node_id, nb_id, weight=c, valid=True)
                connected = True
        return connected

    def _k_nearest(self, graph, x, k, exclude=None):
        """Return k nearest nodes in graph to x (excluding 'exclude' ids)."""
        exclude = exclude or set()
        candidates = [
            (nid, data['x'])
            for nid, data in graph.nodes(data=True)
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
