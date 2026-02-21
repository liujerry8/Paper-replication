#!/usr/bin/env python3
"""
build_roadmap.py

Offline script: generate the PRM* roadmap M from the pre-built approximate
manifold graph G and a reconstructed scene (Section 2.2.4).

The roadmap encodes near-optimal collision-free trajectories for the most
common start/goal pairs in the lab.  It is saved to disk and loaded by the
online LazyPRM planner at query time.

Usage
-----
    rosrun approx_manifold_prm build_roadmap.py \
        --graph /tmp/approx_graph.pkl \
        --output /tmp/roadmap.pkl \
        --max_time 300

Notes
-----
* Collision checking uses the MoveIt! planning scene.  The scene should be
  loaded/reconstructed (e.g. via NeRF-SLAM) before calling this script.
* Run this script once per lab environment.  Re-run only when the environment
  changes significantly.
"""

import argparse
import os
import sys
import numpy as np

try:
    import rospy
    import moveit_commander
    from moveit_commander import PlanningSceneInterface
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from approx_manifold_graph import ApproximateManifoldGraph
from prm_star import PRMStar


# ---------------------------------------------------------------------------
# Collision checker (wraps MoveIt! or uses a stub for offline testing)
# ---------------------------------------------------------------------------

class MoveItCollisionChecker:
    """Check a joint-space configuration for collision via MoveIt!."""

    def __init__(self, move_group):
        self.move_group = move_group

    def is_collision_free(self, joints):
        """Return True if the configuration is collision-free."""
        self.move_group.set_joint_value_target(joints.tolist())
        plan = self.move_group.plan()
        # plan() returns (bool, RobotTrajectory, ...) in MoveIt! 1.x
        if isinstance(plan, tuple):
            return plan[0]
        return plan is not None


class StubCollisionChecker:
    """Stub for testing without ROS: marks all configurations as free."""

    def is_collision_free(self, joints):
        return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Build the PRM* roadmap on an approximate manifold graph'
    )
    parser.add_argument('--graph', type=str, default='/tmp/approx_graph.pkl',
                        help='Path to the pre-built approximate graph (.pkl)')
    parser.add_argument('--output', type=str, default='/tmp/roadmap.pkl',
                        help='Output file path for the roadmap (.pkl)')
    parser.add_argument('--max_time', type=float, default=300.0,
                        help='Maximum PRM* planning time in seconds')
    parser.add_argument('--convergence_threshold', type=float, default=0.10,
                        help='Stop when relative cost change < threshold (default 0.10)')
    args = parser.parse_args()

    move_group = None
    if ROS_AVAILABLE:
        rospy.init_node('build_roadmap', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            move_group = moveit_commander.MoveGroupCommander('manipulator')
            rospy.loginfo('MoveIt! move group initialized.')
        except Exception as e:
            rospy.logwarn(f'Could not initialize MoveIt!: {e}')
    else:
        print('[WARN] ROS not available – using stub collision checker.')

    # Load approximate graph
    from build_approx_graph import UR5eUprightConstraint, UR5E_JOINT_LIMITS
    constraint = UR5eUprightConstraint(move_group=move_group)
    graph = ApproximateManifoldGraph(
        constraint_fn=constraint.constraint_fn,
        jacobian_fn=constraint.jacobian_fn,
        config_dim=6,
        manifold_dim=4,   # k = n - m = 6 - 2 (two constraint equations)
        config_bounds=UR5E_JOINT_LIMITS,
    )
    graph.load(args.graph)
    print(f'Loaded graph: {len(graph.vertices)} vertices')

    # Set up collision checker
    if move_group is not None:
        checker = MoveItCollisionChecker(move_group)
    else:
        checker = StubCollisionChecker()

    # Build PRM* roadmap
    planner = PRMStar(
        approx_graph=graph,
        collision_checker=checker.is_collision_free,
        convergence_threshold=args.convergence_threshold,
    )
    print(f'Building PRM* roadmap (max_time={args.max_time}s)...')
    roadmap = planner.build_roadmap(max_time=args.max_time)
    print(f'Roadmap built: {roadmap.number_of_nodes()} nodes, '
          f'{roadmap.number_of_edges()} edges')

    planner.save(args.output)
    print(f'Roadmap saved to {args.output}')


if __name__ == '__main__':
    main()
