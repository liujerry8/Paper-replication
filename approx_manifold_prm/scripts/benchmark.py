#!/usr/bin/env python3
"""
benchmark.py

Benchmark script that compares the proposed AM-PRM algorithm against a
traditional PRM baseline to verify construction speed and planning quality.

This follows the experimental methodology described in Section 2.3 of the
paper:

  1. Both methods share the same scene, start/goal, and collision checker.
  2. AM-PRM: Build approximate manifold graph → PRM* roadmap → LazyPRM query.
  3. Traditional PRM: Build roadmap by uniform C-space sampling → Dijkstra query.
  4. Each method is run for ``--trials`` independent repetitions.
  5. Results are reported as a comparison table showing:
     - Offline construction time (graph + roadmap build)
     - Online planning time
     - Success rate
     - Path length (joint-space L2 quality metric)

Usage
-----
Minimal (unit-circle toy problem, no ROS required)::

    python3 benchmark.py --mode toy --trials 5

With a scene file (UR5e, no ROS required)::

    python3 benchmark.py --mode scene \\
        --scene scenes/sim_scene_1.yaml \\
        --start "0,0,0,0,0,0" \\
        --goal  "0.3,-0.2,0.1,-0.1,0.1,0" \\
        --n_c 200 --n_e 3 --trials 5

Save results as JSON::

    python3 benchmark.py --mode toy --trials 10 --output /tmp/benchmark.json
"""

import argparse
import json
import os
import sys
import time

import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from approx_manifold_graph import ApproximateManifoldGraph
from prm_star import PRMStar
from lazy_prm import LazyPRM
from traditional_prm import TraditionalPRM


# ---------------------------------------------------------------------------
# Timing utility
# ---------------------------------------------------------------------------

class Timer:
    """Simple context-manager timer that records elapsed seconds."""

    def __init__(self):
        self.elapsed = 0.0

    def __enter__(self):
        self._start = time.perf_counter()
        return self

    def __exit__(self, *args):
        self.elapsed = time.perf_counter() - self._start

    @property
    def elapsed_ms(self):
        return self.elapsed * 1000.0


# ---------------------------------------------------------------------------
# Toy problem helpers (unit circle, same as tests)
# ---------------------------------------------------------------------------

def _circle_F(x):
    return np.array([x[0] ** 2 + x[1] ** 2 - 1.0])


def _circle_J(x):
    return np.array([[2 * x[0], 2 * x[1]]])


_CIRCLE_BOUNDS = np.array([[-2.0, 2.0], [-2.0, 2.0]])


# ---------------------------------------------------------------------------
# UR5e helpers (reused from run_planner.py)
# ---------------------------------------------------------------------------

_UR5E_JOINT_LIMITS = np.array([
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-np.pi,     np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
])


def _upright_F(x, delta=0.01):
    from scene_loader import UR5eFk
    frames = UR5eFk.link_transforms(x)
    R = frames[-1][:3, :3]
    z_col = R[:, 2]
    rx2 = float((1.0 - z_col[2]) ** 2)
    ry2 = float(z_col[0] ** 2 + z_col[1] ** 2)
    return np.array([max(0.0, rx2 - delta), max(0.0, ry2 - delta)])


def _upright_J(x, delta=0.01, eps=1e-5):
    f0 = _upright_F(x, delta)
    J = np.zeros((len(f0), len(x)))
    for i in range(len(x)):
        xp = x.copy()
        xp[i] += eps
        J[:, i] = (_upright_F(xp, delta) - f0) / eps
    return J


# ---------------------------------------------------------------------------
# Single-trial runners
# ---------------------------------------------------------------------------

def run_am_prm_trial(constraint_fn, jacobian_fn, config_dim, manifold_dim,
                     config_bounds, collision_fn, edge_checker,
                     x_start, x_goal, n_c, n_e, roadmap_time, plan_time):
    """
    Run a single AM-PRM trial and return timing + result metrics.

    Returns
    -------
    dict with keys: graph_build_s, roadmap_build_s, plan_ms, success,
                    path_length, n_vertices, n_roadmap_nodes, n_roadmap_edges
    """
    result = {}

    # Phase 1: build approximate manifold graph
    graph = ApproximateManifoldGraph(
        constraint_fn=constraint_fn,
        jacobian_fn=jacobian_fn,
        config_dim=config_dim,
        manifold_dim=manifold_dim,
        config_bounds=config_bounds,
    )
    with Timer() as t_graph:
        graph.build(n_c=n_c, n_e=n_e)
    result['graph_build_s'] = t_graph.elapsed

    # Phase 2: build PRM* roadmap on the manifold graph
    prm = PRMStar(approx_graph=graph, collision_checker=collision_fn)
    with Timer() as t_roadmap:
        roadmap = prm.build_roadmap(max_time=roadmap_time)
    result['roadmap_build_s'] = t_roadmap.elapsed

    result['n_vertices'] = len(graph.vertices)
    result['n_roadmap_nodes'] = roadmap.number_of_nodes()
    result['n_roadmap_edges'] = roadmap.number_of_edges()

    # Phase 3: online LazyPRM query
    lazy = LazyPRM(
        roadmap=roadmap,
        collision_checker=collision_fn,
        edge_checker=edge_checker,
        connect_k=10,
    )
    path, t_ms = lazy.plan(x_start, x_goal, time_limit=plan_time)

    result['plan_ms'] = t_ms
    result['success'] = path is not None
    result['path_length'] = _path_length(path) if path is not None else None
    result['offline_total_s'] = result['graph_build_s'] + result['roadmap_build_s']

    return result


def run_traditional_prm_trial(config_dim, config_bounds, collision_fn,
                              constraint_fn, jacobian_fn,
                              x_start, x_goal, n_samples, max_time):
    """
    Run a single Traditional PRM trial and return timing + result metrics.

    Returns
    -------
    dict with keys: roadmap_build_s, plan_ms, success, path_length,
                    n_roadmap_nodes, n_roadmap_edges
    """
    result = {}

    planner = TraditionalPRM(
        config_dim=config_dim,
        config_bounds=config_bounds,
        collision_checker=collision_fn,
        constraint_fn=constraint_fn,
        jacobian_fn=jacobian_fn,
    )

    roadmap, build_time = planner.build_roadmap(
        n_samples=n_samples, max_time=max_time)

    result['roadmap_build_s'] = build_time
    result['n_roadmap_nodes'] = roadmap.number_of_nodes()
    result['n_roadmap_edges'] = roadmap.number_of_edges()

    path, cost, t_ms = planner.query(x_start, x_goal)
    result['plan_ms'] = t_ms
    result['success'] = path is not None
    result['path_length'] = _path_length(path) if path is not None else None
    result['offline_total_s'] = result['roadmap_build_s']

    return result


def _path_length(path):
    if path is None or len(path) < 2:
        return 0.0
    return sum(
        float(np.linalg.norm(np.array(path[i + 1]) - np.array(path[i])))
        for i in range(len(path) - 1)
    )


# ---------------------------------------------------------------------------
# Statistics helpers
# ---------------------------------------------------------------------------

def compute_stats(trials):
    """Compute aggregate statistics from a list of trial result dicts."""
    n = len(trials)
    if n == 0:
        return {}
    successes = sum(1 for t in trials if t['success'])

    def _agg(key):
        vals = [t[key] for t in trials if t[key] is not None]
        if not vals:
            return {'mean': None, 'std': None, 'min': None, 'max': None}
        return {
            'mean': float(np.mean(vals)),
            'std': float(np.std(vals)),
            'min': float(np.min(vals)),
            'max': float(np.max(vals)),
        }

    stats = {
        'trials': n,
        'success_rate': successes / n,
        'offline_time_s': _agg('offline_total_s'),
        'plan_time_ms': _agg('plan_ms'),
        'path_length': _agg('path_length'),
    }

    if 'graph_build_s' in trials[0]:
        stats['graph_build_s'] = _agg('graph_build_s')
        stats['roadmap_build_s'] = _agg('roadmap_build_s')
    else:
        stats['roadmap_build_s'] = _agg('roadmap_build_s')

    return stats


def print_comparison_table(am_stats, trad_stats):
    """Print a formatted comparison table to stdout."""
    print()
    print("=" * 72)
    print("  Benchmark Comparison: AM-PRM vs Traditional PRM")
    print("=" * 72)

    hdr = f"  {'Metric':<30s} {'AM-PRM':>18s} {'Traditional PRM':>18s}"
    print(hdr)
    print("  " + "-" * 68)

    def _fmt(stats, key, unit=''):
        agg = stats.get(key, {})
        if isinstance(agg, dict):
            m = agg.get('mean')
            s = agg.get('std')
            if m is None:
                return 'N/A'
            return f"{m:.2f} ± {s:.2f}{unit}"
        return str(agg)

    rows = [
        ('Trials', str(am_stats['trials']), str(trad_stats['trials'])),
        ('Success Rate', f"{am_stats['success_rate']*100:.1f}%",
         f"{trad_stats['success_rate']*100:.1f}%"),
        ('Offline Build Time (s)', _fmt(am_stats, 'offline_time_s'),
         _fmt(trad_stats, 'offline_time_s')),
        ('Online Plan Time (ms)', _fmt(am_stats, 'plan_time_ms'),
         _fmt(trad_stats, 'plan_time_ms')),
        ('Path Length (rad)', _fmt(am_stats, 'path_length'),
         _fmt(trad_stats, 'path_length')),
    ]

    if 'graph_build_s' in am_stats:
        rows.insert(2, (
            '  Graph Build Time (s)',
            _fmt(am_stats, 'graph_build_s'), 'N/A'))
        rows.insert(3, (
            '  Roadmap Build Time (s)',
            _fmt(am_stats, 'roadmap_build_s'),
            _fmt(trad_stats, 'roadmap_build_s')))

    for label, am_val, trad_val in rows:
        print(f"  {label:<30s} {am_val:>18s} {trad_val:>18s}")

    print("=" * 72)
    print()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_joints(s):
    vals = [float(v.strip()) for v in s.split(',')]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError(
            f"Expected 6 joint values, got {len(vals)}: {s!r}")
    return np.array(vals)


def main():
    parser = argparse.ArgumentParser(
        description='Benchmark AM-PRM vs Traditional PRM',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--mode', choices=['toy', 'scene'], default='toy',
        help='Benchmark mode: "toy" (unit-circle 2-D) or "scene" (UR5e + YAML)')
    parser.add_argument('--scene', default=None,
                        help='YAML scene file (required when mode=scene)')
    parser.add_argument('--start', default=None, type=parse_joints,
                        help='Start joints for scene mode')
    parser.add_argument('--goal', default=None, type=parse_joints,
                        help='Goal joints for scene mode')
    parser.add_argument('--n_c', type=int, default=200,
                        help='AM-PRM graph vertices (default 200)')
    parser.add_argument('--n_e', type=int, default=3,
                        help='AM-PRM max edges per vertex (default 3)')
    parser.add_argument('--n_samples', type=int, default=None,
                        help='Traditional PRM samples (default = n_c)')
    parser.add_argument('--roadmap_time', type=float, default=10.0,
                        help='AM-PRM roadmap build time budget (s)')
    parser.add_argument('--plan_time', type=float, default=10.0,
                        help='Online planning time limit (s)')
    parser.add_argument('--trad_build_time', type=float, default=30.0,
                        help='Traditional PRM build time budget (s)')
    parser.add_argument('--trials', type=int, default=5,
                        help='Number of independent trials per method')
    parser.add_argument('--seed', type=int, default=None,
                        help='Base random seed (trial i uses seed+i)')
    parser.add_argument('--output', default=None,
                        help='Save results as JSON to this path')
    args = parser.parse_args()

    if args.n_samples is None:
        args.n_samples = args.n_c

    # ---- Setup -----------------------------------------------------------
    if args.mode == 'toy':
        constraint_fn = _circle_F
        jacobian_fn = _circle_J
        config_dim = 2
        manifold_dim = 1
        config_bounds = _CIRCLE_BOUNDS
        collision_fn = lambda x: True  # no obstacles
        edge_checker = lambda xa, xb: True

        # Start/goal on the unit circle
        x_start = np.array([1.0, 0.0])
        x_goal = np.array([-1.0, 0.0])

    elif args.mode == 'scene':
        if args.scene is None or args.start is None or args.goal is None:
            parser.error('--scene, --start, and --goal are required '
                         'when mode=scene')
        from scene_loader import SceneLoader, SceneCollisionChecker
        scene_path = os.path.abspath(args.scene)
        objects, meta = SceneLoader.from_yaml(scene_path)
        print(f"Scene '{meta.get('name', scene_path)}': "
              f"{len(objects)} obstacle(s) loaded")
        checker = SceneCollisionChecker(objects)
        collision_fn = checker.is_free

        def edge_checker(xa, xb, n_steps=10):
            for t in np.linspace(0, 1, n_steps + 2)[1:-1]:
                x = (1 - t) * xa + t * xb
                if not collision_fn(x):
                    return False
            return True

        constraint_fn = _upright_F
        jacobian_fn = _upright_J
        config_dim = 6
        manifold_dim = 4
        config_bounds = _UR5E_JOINT_LIMITS
        x_start = args.start
        x_goal = args.goal

    # ---- Run trials ------------------------------------------------------
    am_trials = []
    trad_trials = []

    for i in range(args.trials):
        seed = (args.seed + i) if args.seed is not None else None
        if seed is not None:
            np.random.seed(seed)

        print(f"\n--- Trial {i+1}/{args.trials} ---")

        # AM-PRM
        print("  Running AM-PRM …")
        am_result = run_am_prm_trial(
            constraint_fn=constraint_fn,
            jacobian_fn=jacobian_fn,
            config_dim=config_dim,
            manifold_dim=manifold_dim,
            config_bounds=config_bounds,
            collision_fn=collision_fn,
            edge_checker=edge_checker,
            x_start=x_start,
            x_goal=x_goal,
            n_c=args.n_c,
            n_e=args.n_e,
            roadmap_time=args.roadmap_time,
            plan_time=args.plan_time,
        )
        status = "OK" if am_result['success'] else "FAIL"
        print(f"    Graph: {am_result['graph_build_s']:.2f}s, "
              f"Roadmap: {am_result['roadmap_build_s']:.2f}s, "
              f"Plan: {am_result['plan_ms']:.1f}ms [{status}]")
        am_trials.append(am_result)

        # Traditional PRM (use same seed so sampling is independent but reproducible)
        if seed is not None:
            np.random.seed(seed + 1000)

        print("  Running Traditional PRM …")
        trad_result = run_traditional_prm_trial(
            config_dim=config_dim,
            config_bounds=config_bounds,
            collision_fn=collision_fn,
            constraint_fn=constraint_fn,
            jacobian_fn=jacobian_fn,
            x_start=x_start,
            x_goal=x_goal,
            n_samples=args.n_samples,
            max_time=args.trad_build_time,
        )
        status = "OK" if trad_result['success'] else "FAIL"
        print(f"    Roadmap: {trad_result['roadmap_build_s']:.2f}s, "
              f"Plan: {trad_result['plan_ms']:.1f}ms [{status}]")
        trad_trials.append(trad_result)

    # ---- Aggregate & report ----------------------------------------------
    am_stats = compute_stats(am_trials)
    trad_stats = compute_stats(trad_trials)
    print_comparison_table(am_stats, trad_stats)

    # ---- Save output -----------------------------------------------------
    if args.output:
        out = {
            'mode': args.mode,
            'trials': args.trials,
            'am_prm': {'trials': am_trials, 'stats': am_stats},
            'traditional_prm': {'trials': trad_trials, 'stats': trad_stats},
        }
        with open(args.output, 'w') as f:
            json.dump(out, f, indent=2, default=str)
        print(f"Results saved to {args.output!r}")


if __name__ == '__main__':
    main()
