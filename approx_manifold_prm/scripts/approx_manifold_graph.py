#!/usr/bin/env python3
"""
approx_manifold_graph.py

Implements the Approximate Manifold Graph construction algorithm described in:
"基于近似流形概率路线图的机器人受约束轨迹规划" (Chapter 2).

The approximate manifold graph G = (V, E) is an undirected graph that
approximates the constraint manifold X = {x in A | F(x) = 0}.

Key steps:
  1. Project configurations onto the constraint manifold using Newton-Raphson.
  2. Validate diversity of new configurations using equations (2.6)-(2.8).
  3. Build edge set by connecting nearby configurations that stay on the manifold.
"""

import numpy as np
import pickle
from scipy.spatial import cKDTree


class ApproximateManifoldGraph:
    """
    Undirected graph approximating a robot constraint manifold X.

    Parameters
    ----------
    constraint_fn : callable
        F(x) -> R^(n-k).  Constraint function; a configuration x is on the
        manifold when F(x) == 0.
    jacobian_fn : callable
        J(x) -> R^((n-k) x n).  Jacobian of the constraint function.
    config_dim : int
        Dimension n of the full configuration space A.
    manifold_dim : int
        Dimension k of the constraint manifold.
        k = n - m, where m = len(F(x)) is the number of independent
        constraint equations.  E.g. for a 6-DOF arm with 2 equality
        constraints, manifold_dim = 6 - 2 = 4.
    config_bounds : array-like of shape (n, 2)
        Lower and upper bounds for each configuration DOF.
    projection_tol : float
        Convergence tolerance for Newton-Raphson projection (||F(x)|| < tol).
    projection_max_iter : int
        Maximum Newton-Raphson iterations.
    diversity_epsilon : float
        Minimum distance to tangent plane (eq. 2.6).
    diversity_rho : float
        Minimum distance in tangent coordinates (eq. 2.8).
    diversity_alpha : float
        Maximum tangent-space alignment angle in radians (eq. 2.7).
    edge_validity_zeta : float
        Maximum constraint violation allowed along an edge (eq. 2.11).
    edge_validity_samples : int
        Number of samples along an edge for validity check.
    """

    def __init__(
        self,
        constraint_fn,
        jacobian_fn,
        config_dim,
        manifold_dim,
        config_bounds,
        projection_tol=1e-4,
        projection_max_iter=50,
        diversity_epsilon=0.05,
        diversity_rho=0.05,
        diversity_alpha=np.pi / 6,
        edge_validity_zeta=1e-3,
        edge_validity_samples=10,
    ):
        self.F = constraint_fn
        self.J = jacobian_fn
        self.n = config_dim
        self.k = manifold_dim
        self.bounds = np.array(config_bounds)
        self.proj_tol = projection_tol
        self.proj_max_iter = projection_max_iter
        self.eps = diversity_epsilon
        self.rho = diversity_rho
        self.alpha = diversity_alpha
        self.zeta = edge_validity_zeta
        self.edge_samples = edge_validity_samples

        self.vertices = []          # list of np.ndarray  (configurations)
        self.tangent_bases = []     # list of np.ndarray  (Phi_x at each vertex)
        self.adjacency = {}         # dict: int -> set of int

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def build(self, n_c, n_e):
        """
        Build the approximate manifold graph (Algorithm 2.1 + 2.2).

        Parameters
        ----------
        n_c : int
            Target number of vertices in the graph.
        n_e : int
            Maximum number of edges per vertex (0 = no edges).
        """
        self._build_vertex_set(n_c)
        if n_e > 0:
            self._build_edge_set(n_e)

    def sample_uniform(self):
        """
        Uniform random sampling on the approximate graph.

        Returns a uniformly chosen configuration from the vertex set V.
        """
        idx = np.random.randint(0, len(self.vertices))
        return self.vertices[idx], idx

    def sample_nearby(self, x, d):
        """
        Sample a configuration within distance d of x on the manifold.

        If x is in V and has connected neighbours within distance d, returns
        one of those neighbours uniformly at random.  Otherwise samples a
        random configuration in the ball of radius d around x and projects
        it onto the manifold.

        Parameters
        ----------
        x : np.ndarray
            Query configuration (must be on the manifold).
        d : float
            Maximum distance.

        Returns
        -------
        x_sample : np.ndarray | None
            A nearby configuration on the manifold, or None on failure.
        """
        if len(self.vertices) == 0:
            return None

        # Try to find vertex x in V
        idx = self._find_vertex(x)
        if idx is not None:
            neighbours = [
                j for j in self.adjacency.get(idx, set())
                if np.linalg.norm(self.vertices[j] - x) <= d
            ]
            if neighbours:
                j = np.random.choice(neighbours)
                return self.vertices[j]

        # Fallback: sample in ball and project
        for _ in range(50):
            direction = np.random.randn(self.n)
            direction /= np.linalg.norm(direction)
            r = np.random.uniform(0, d)
            x_rand = x + r * direction
            x_proj = self.project(x_rand)
            if x_proj is not None:
                return x_proj
        return None

    def project(self, x0):
        """
        Project a configuration onto the constraint manifold (eq. 2.5).

        Uses Newton-Raphson iteration:  x_{i+1} = x_i - J(x_i)^+ F(x_i)

        Parameters
        ----------
        x0 : np.ndarray
            Initial configuration in the full space A.

        Returns
        -------
        x : np.ndarray | None
            Configuration on the manifold, or None if projection failed.
        """
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

    def tangent_basis(self, x):
        """
        Compute an orthonormal basis Phi_x for the tangent space at x (eq. 2.2).

        Phi_x = null space of J(x), i.e. the k-dimensional space orthogonal
        to the constraint gradients.

        Returns
        -------
        Phi : np.ndarray of shape (n, k)
        """
        J_val = self.J(x)
        _, _, Vt = np.linalg.svd(J_val, full_matrices=True)
        # J has shape (m, n) where m = n - k.
        # The last k rows of Vt (the full right singular matrix) span the
        # null space of J, i.e. the tangent space of the manifold at x.
        Phi = Vt[-self.k:].T  # shape (n, k)
        return Phi

    def save(self, path):
        """Serialize the graph to a file."""
        data = {
            'vertices': self.vertices,
            'tangent_bases': self.tangent_bases,
            'adjacency': self.adjacency,
            'params': {
                'n': self.n, 'k': self.k,
                'eps': self.eps, 'rho': self.rho,
                'alpha': self.alpha, 'zeta': self.zeta,
            }
        }
        with open(path, 'wb') as f:
            pickle.dump(data, f)

    def load(self, path):
        """Deserialize the graph from a file."""
        with open(path, 'rb') as f:
            data = pickle.load(f)
        self.vertices = data['vertices']
        self.tangent_bases = data['tangent_bases']
        self.adjacency = data['adjacency']

    # ------------------------------------------------------------------
    # Algorithm 2.1 – Build vertex set V
    # ------------------------------------------------------------------

    def _build_vertex_set(self, n_c):
        """Construct the vertex set V (Algorithm 2.1)."""
        self.vertices = []
        self.tangent_bases = []
        self.adjacency = {}
        max_attempts = n_c * 200
        attempts = 0

        while len(self.vertices) < n_c and attempts < max_attempts:
            attempts += 1
            x0 = self._random_config()
            x = self.project(x0)
            if x is None:
                continue
            Phi = self.tangent_basis(x)
            if self._diversity_check(x, Phi):
                idx = len(self.vertices)
                self.vertices.append(x)
                self.tangent_bases.append(Phi)
                self.adjacency[idx] = set()

        if len(self.vertices) < n_c:
            import warnings
            warnings.warn(
                f"Only {len(self.vertices)}/{n_c} vertices found after "
                f"{max_attempts} attempts. Consider relaxing diversity parameters."
            )

    # ------------------------------------------------------------------
    # Algorithm 2.2 – Build edge set E
    # ------------------------------------------------------------------

    def _build_edge_set(self, n_e):
        """Construct the edge set E (Algorithm 2.2)."""
        if not self.vertices:
            return
        V_arr = np.array(self.vertices)
        tree = cKDTree(V_arr)

        for i, v in enumerate(self.vertices):
            if len(self.adjacency[i]) >= n_e:
                continue
            # Query more neighbours than n_e because some may fail validity
            k_query = min(n_e * 3 + 1, len(self.vertices))
            dists, idxs = tree.query(v, k=k_query)
            for dist, j in zip(dists, idxs):
                if j == i:
                    continue
                if len(self.adjacency[i]) >= n_e:
                    break
                if j in self.adjacency[i]:
                    continue
                if self._valid_edge(v, self.vertices[j]):
                    self.adjacency[i].add(j)
                    self.adjacency[j].add(i)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _random_config(self):
        """Sample a random configuration uniformly within bounds."""
        lo, hi = self.bounds[:, 0], self.bounds[:, 1]
        return lo + np.random.rand(self.n) * (hi - lo)

    def _diversity_check(self, x_i, Phi_i):
        """
        Check that x_i is sufficiently different from every x_j in V.

        Equations (2.6)-(2.8): all three inequalities must hold for x_i to be
        accepted into V.

        Returns True when x_i is diverse enough (valid), False otherwise.
        """
        for x_j, Phi_j in zip(self.vertices, self.tangent_bases):
            diff = x_j - x_i
            u_ij = Phi_i.T @ diff                         # tangent coords
            dist_to_plane = np.linalg.norm(diff - Phi_i @ u_ij)  # eq. 2.6 LHS
            align = np.linalg.norm(Phi_i.T @ Phi_j)      # eq. 2.7 LHS
            tangent_dist = np.linalg.norm(u_ij)           # eq. 2.8 LHS

            # x_i is NOT diverse if ANY of the three conditions fails
            if not (
                dist_to_plane > self.eps          # eq. 2.6
                and align < np.cos(self.alpha)    # eq. 2.7
                and tangent_dist > self.rho       # eq. 2.8
            ):
                return False
        return True

    def _valid_edge(self, x_k, x_n):
        """
        Check if the straight-line segment x_k -> x_n stays on the manifold.

        Equation (2.11): all interpolated configurations must satisfy
        ||F(x_l)|| <= zeta.
        """
        for t in np.linspace(0, 1, self.edge_samples + 2)[1:-1]:
            x_l = (1 - t) * x_k + t * x_n
            if np.linalg.norm(self.F(x_l)) > self.zeta:
                return False
        return True

    def _find_vertex(self, x, tol=1e-6):
        """Return the index of x in V, or None if not found."""
        for i, v in enumerate(self.vertices):
            if np.linalg.norm(v - x) < tol:
                return i
        return None
