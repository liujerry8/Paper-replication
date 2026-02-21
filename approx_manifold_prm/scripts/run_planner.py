#!/usr/bin/env python3
"""
run_planner.py

End-to-end command-line script: given a scene file, run the full
Approximate Manifold PRM pipeline and report the planned trajectory.

No ROS installation is required.  The script uses geometric collision
checking (UR5e FK + link capsules) against the obstacles defined in the
scene YAML file.

Usage
-----
Minimal (auto-builds graph and roadmap each time)::

    python3 run_planner.py \\
        --scene  scenes/sim_scene_1.yaml \\
        --start  "0,0,0,0,0,0" \\
        --goal   "1.57,-1.0,0.5,-1.0,0.5,0"

Faster (reuse a pre-built graph and roadmap)::

    python3 run_planner.py \\
        --scene      scenes/sim_scene_1.yaml \\
        --graph      /tmp/approx_graph.pkl \\
        --roadmap    /tmp/roadmap.pkl \\
        --start      "0,0,0,0,0,0" \\
        --goal       "1.57,-1.0,0.5,-1.0,0.5,0"

Build and save graph + roadmap for later reuse::

    python3 run_planner.py \\
        --scene         scenes/sim_scene_1.yaml \\
        --start         "0,0,0,0,0,0" \\
        --goal          "1.57,-1.0,0.5,-1.0,0.5,0" \\
        --save_graph    /tmp/approx_graph.pkl \\
        --save_roadmap  /tmp/roadmap.pkl

Output
------
The planned joint-space path is printed as a table of waypoints.
Use ``--output path.json`` to also save it as a JSON file.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

# Allow importing sibling scripts without a full catkin install
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from approx_manifold_graph import ApproximateManifoldGraph
from prm_star import PRMStar
from lazy_prm import LazyPRM
from scene_loader import SceneLoader, SceneCollisionChecker

# UR5e joint limits (radians) – re-used from build_approx_graph.py
_UR5E_JOINT_LIMITS = np.array([
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-np.pi,     np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
    [-2 * np.pi, 2 * np.pi],
])


# ---------------------------------------------------------------------------
# UR5e upright constraint (same logic as build_approx_graph.py)
# ---------------------------------------------------------------------------

def _upright_F(x, delta=0.01):
    """Inequality constraint as ReLU equality (2 outputs)."""
    from scene_loader import UR5eFk
    frames = UR5eFk.link_transforms(x)
    R = frames[-1][:3, :3]
    # r_x = atan2(R[2,1], R[2,2]),  r_y = atan2(-R[2,0], sqrt(...))
    # Approximate: use z-column of end-effector rotation.
    # A perfectly upright tool has z-column pointing up: R[:,2] ≈ [0,0,1].
    # We use deviation from z-axis as the constraint.
    z_col = R[:, 2]           # end-effector Z axis in world frame
    rx2 = float((1.0 - z_col[2]) ** 2)   # 0 when z_col == [0,0,1]
    ry2 = float(z_col[0] ** 2 + z_col[1] ** 2)
    return np.array([
        max(0.0, rx2 - delta),
        max(0.0, ry2 - delta),
    ])


def _upright_J(x, delta=0.01, eps=1e-5):
    """Numerical Jacobian of _upright_F."""
    f0 = _upright_F(x, delta)
    J = np.zeros((len(f0), len(x)))
    for i in range(len(x)):
        xp = x.copy()
        xp[i] += eps
        J[:, i] = (_upright_F(xp, delta) - f0) / eps
    return J


# ---------------------------------------------------------------------------
# Build helpers
# ---------------------------------------------------------------------------

def build_graph(collision_fn, n_c=500, n_e=3, verbose=True):
    """Construct the approximate manifold graph."""
    graph = ApproximateManifoldGraph(
        constraint_fn=_upright_F,
        jacobian_fn=_upright_J,
        config_dim=6,
        manifold_dim=4,
        config_bounds=_UR5E_JOINT_LIMITS,
    )
    if verbose:
        print(f"Building approximate manifold graph (n_c={n_c}, n_e={n_e}) …")
    t0 = time.time()
    graph.build(n_c=n_c, n_e=n_e)
    if verbose:
        print(f"  Done in {time.time()-t0:.1f}s: "
              f"{len(graph.vertices)} vertices, "
              f"{sum(len(v) for v in graph.adjacency.values())//2} edges")
    return graph


def build_roadmap(graph, collision_fn, max_time=30.0, verbose=True):
    """Build the PRM* roadmap."""
    planner = PRMStar(
        approx_graph=graph,
        collision_checker=collision_fn,
    )
    if verbose:
        print(f"Building PRM* roadmap (max_time={max_time}s) …")
    t0 = time.time()
    roadmap = planner.build_roadmap(max_time=max_time)
    if verbose:
        print(f"  Done in {time.time()-t0:.1f}s: "
              f"{roadmap.number_of_nodes()} nodes, "
              f"{roadmap.number_of_edges()} edges")
    return roadmap


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def parse_joints(s):
    """Parse a comma-separated string of 6 joint values in radians."""
    vals = [float(v.strip()) for v in s.split(',')]
    if len(vals) != 6:
        raise argparse.ArgumentTypeError(
            f"Expected 6 joint values, got {len(vals)}: {s!r}"
        )
    return np.array(vals)


def main():
    parser = argparse.ArgumentParser(
        description='Run the Approximate Manifold PRM planner on a scene file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        '--scene', required=True,
        help='Path to the YAML scene file describing obstacles'
    )
    parser.add_argument(
        '--start', required=True, type=parse_joints,
        help='Start joint configuration as comma-separated radians, e.g. "0,0,0,0,0,0"'
    )
    parser.add_argument(
        '--goal', required=True, type=parse_joints,
        help='Goal joint configuration as comma-separated radians'
    )
    parser.add_argument(
        '--graph', default=None,
        help='Path to a pre-built approximate graph (.pkl).  '
             'If omitted the graph is built from scratch.'
    )
    parser.add_argument(
        '--roadmap', default=None,
        help='Path to a pre-built PRM* roadmap (.pkl).  '
             'If omitted the roadmap is built from scratch.'
    )
    parser.add_argument(
        '--save_graph', default=None,
        help='Save the (newly built) approximate graph to this path.'
    )
    parser.add_argument(
        '--save_roadmap', default=None,
        help='Save the (newly built) roadmap to this path.'
    )
    parser.add_argument(
        '--n_c', type=int, default=500,
        help='Number of graph vertices when building from scratch (default 500)'
    )
    parser.add_argument(
        '--n_e', type=int, default=3,
        help='Max edges per vertex when building graph from scratch (default 3)'
    )
    parser.add_argument(
        '--graph_time', type=float, default=30.0,
        help='Max seconds for PRM* roadmap build (default 30)'
    )
    parser.add_argument(
        '--plan_time', type=float, default=10.0,
        help='Max seconds for online LazyPRM query (default 10)'
    )
    parser.add_argument(
        '--output', default=None,
        help='Save the planned path to this JSON file'
    )
    parser.add_argument(
        '--delta', type=float, default=0.01,
        help='Upright constraint threshold r_x^2, r_y^2 <= delta (default 0.01)'
    )
    args = parser.parse_args()

    # 1. Load scene ─────────────────────────────────────────────────────────
    scene_path = os.path.abspath(args.scene)
    if not os.path.isfile(scene_path):
        print(f"[ERROR] Scene file not found: {scene_path}")
        sys.exit(1)

    objects, meta = SceneLoader.from_yaml(scene_path)
    print(f"Scene '{meta.get('name', scene_path)}': "
          f"{len(objects)} obstacle(s) loaded")
    for obj in objects:
        print(f"  • {obj.name} ({obj.type})")

    checker = SceneCollisionChecker(objects)

    # 2. Check that start and goal are collision-free ────────────────────────
    for label, joints in [('start', args.start), ('goal', args.goal)]:
        if not checker.is_free(joints):
            print(f"[ERROR] {label.capitalize()} configuration is in collision. "
                  "Please choose a different configuration.")
            sys.exit(1)
    print("Start and goal configurations are collision-free ✓")

    # 3. Load or build the approximate manifold graph ────────────────────────
    if args.graph and os.path.isfile(args.graph):
        print(f"Loading approximate graph from {args.graph!r} …")
        graph = ApproximateManifoldGraph(
            constraint_fn=_upright_F,
            jacobian_fn=_upright_J,
            config_dim=6,
            manifold_dim=4,
            config_bounds=_UR5E_JOINT_LIMITS,
        )
        graph.load(args.graph)
        print(f"  Loaded: {len(graph.vertices)} vertices")
    else:
        graph = build_graph(
            collision_fn=checker.is_free,
            n_c=args.n_c,
            n_e=args.n_e,
        )
        if args.save_graph:
            graph.save(args.save_graph)
            print(f"  Graph saved to {args.save_graph!r}")

    # 4. Load or build the roadmap ───────────────────────────────────────────
    import pickle
    if args.roadmap and os.path.isfile(args.roadmap):
        print(f"Loading roadmap from {args.roadmap!r} …")
        with open(args.roadmap, 'rb') as f:
            roadmap = pickle.load(f)
        print(f"  Loaded: {roadmap.number_of_nodes()} nodes")
    else:
        roadmap = build_roadmap(
            graph=graph,
            collision_fn=checker.is_free,
            max_time=args.graph_time,
        )
        if args.save_roadmap:
            planner_tmp = PRMStar(graph, checker.is_free)
            planner_tmp.roadmap = roadmap
            planner_tmp.save(args.save_roadmap)
            print(f"  Roadmap saved to {args.save_roadmap!r}")

    # 5. Online LazyPRM query ────────────────────────────────────────────────
    def edge_checker(x_a, x_b, n_steps=10):
        for t in np.linspace(0, 1, n_steps + 2)[1:-1]:
            x = (1 - t) * x_a + t * x_b
            if not checker.is_free(x):
                return False
        return True

    lazy = LazyPRM(
        roadmap=roadmap,
        collision_checker=checker.is_free,
        edge_checker=edge_checker,
        connect_k=10,
    )

    print(f"\nPlanning from start={np.round(args.start, 3).tolist()} "
          f"to goal={np.round(args.goal, 3).tolist()} …")
    path, t_ms = lazy.plan(args.start, args.goal, time_limit=args.plan_time)

    # 6. Report results ──────────────────────────────────────────────────────
    if path is None:
        print(f"\n[FAIL] No path found in {t_ms:.1f} ms.")
        print("Tips:")
        print("  • Increase --n_c (more graph vertices)")
        print("  • Increase --graph_time (more PRM* iterations)")
        print("  • Check that start and goal are reachable in the scene")
        sys.exit(2)

    print(f"\n[SUCCESS] Path found in {t_ms:.1f} ms, {len(path)} waypoints:")
    print(f"  {'WP':>3}  {'j1':>8}  {'j2':>8}  {'j3':>8}"
          f"  {'j4':>8}  {'j5':>8}  {'j6':>8}")
    print("  " + "-" * 62)
    for i, wp in enumerate(path):
        vals = "  ".join(f"{v:8.4f}" for v in wp)
        print(f"  {i:>3}  {vals}")

    total_length = sum(
        float(np.linalg.norm(path[i+1] - path[i]))
        for i in range(len(path) - 1)
    )
    print(f"\nPath length (joint-space L2): {total_length:.4f} rad")

    # 7. Save output ─────────────────────────────────────────────────────────
    if args.output:
        out = {
            'scene': scene_path,
            'start': args.start.tolist(),
            'goal': args.goal.tolist(),
            'planning_time_ms': t_ms,
            'waypoints': [wp.tolist() for wp in path],
            'path_length': total_length,
        }
        with open(args.output, 'w') as f:
            json.dump(out, f, indent=2)
        print(f"\nPath saved to {args.output!r}")


if __name__ == '__main__':
    main()
