#!/usr/bin/env python3
"""
build_approx_graph.py

Offline script: build the approximate manifold graph G = (V, E) for a
UR5e robot arm with the end-effector pose constraint described in Section 2.3.

Constraint (inequality, relaxed from equality):
    r_x^2 <= 0.01  and  r_y^2 <= 0.01

where r_x, r_y are the end-effector rotation angles around X and Y axes.
This keeps the held vessel (test tube rack / reagent bottle) upright.

Usage
-----
    rosrun approx_manifold_prm build_approx_graph.py \
        --n_c 1000 --n_e 5 --output /tmp/approx_graph.pkl

The graph file is then consumed by build_roadmap.py and the online planner.
"""

import argparse
import os
import sys
import numpy as np

# ROS / MoveIt!  (only imported when running inside a ROS environment)
try:
    import rospy
    import moveit_commander
    from geometry_msgs.msg import Pose
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

# Local modules (work both with and without catkin)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from approx_manifold_graph import ApproximateManifoldGraph


# ---------------------------------------------------------------------------
# UR5e joint limits  (radians)
# ---------------------------------------------------------------------------
UR5E_JOINT_LIMITS = np.array([
    [-2 * np.pi, 2 * np.pi],   # shoulder_pan_joint
    [-2 * np.pi, 2 * np.pi],   # shoulder_lift_joint
    [-np.pi,     np.pi],        # elbow_joint
    [-2 * np.pi, 2 * np.pi],   # wrist_1_joint
    [-2 * np.pi, 2 * np.pi],   # wrist_2_joint
    [-2 * np.pi, 2 * np.pi],   # wrist_3_joint
])


# ---------------------------------------------------------------------------
# Constraint functions (requires MoveIt! for FK)
# ---------------------------------------------------------------------------

class UR5eUprightConstraint:
    """
    End-effector upright constraint for UR5e.

    F(x) encodes the soft constraint r_x^2 + r_y^2 <= delta.

    When using MoveIt! FK we get the full pose; when MoveIt! is unavailable
    a placeholder (identity) FK is used so that the graph structure can still
    be tested offline.
    """

    def __init__(self, move_group=None, delta=0.01):
        self.move_group = move_group
        self.delta = delta

    def fk(self, joints):
        """Return (r_x, r_y) end-effector roll/pitch from joint angles."""
        if self.move_group is None:
            # Placeholder FK for testing without ROS: returns small random values
            return np.zeros(2)
        self.move_group.set_joint_value_target(joints.tolist())
        pose: Pose = self.move_group.get_current_pose().pose
        # Convert quaternion to roll/pitch
        from tf.transformations import euler_from_quaternion
        q = [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        roll, pitch, _ = euler_from_quaternion(q)
        return np.array([roll, pitch])

    def constraint_fn(self, x):
        """
        F(x): inequality constraint encoded as equality via ReLU.

        Returns a 2-vector whose zero-set is the feasible manifold:
            [max(0, r_x^2 - delta), max(0, r_y^2 - delta)]

        Note: ReLU introduces a non-smooth boundary at r_i^2 == delta.
        Newton-Raphson may converge slowly for configurations right at
        the boundary.  In practice this is rare; increasing projection_tol
        slightly (e.g. 1e-3) avoids excessive iterations near the boundary.
        """
        r = self.fk(x)
        return np.array([
            max(0.0, r[0] ** 2 - self.delta),
            max(0.0, r[1] ** 2 - self.delta),
        ])

    def jacobian_fn(self, x):
        """
        Numerical Jacobian of F with respect to joint angles x.
        Shape: (2, 6)
        """
        eps = 1e-5
        f0 = self.constraint_fn(x)
        J = np.zeros((len(f0), len(x)))
        for i in range(len(x)):
            x_plus = x.copy()
            x_plus[i] += eps
            J[:, i] = (self.constraint_fn(x_plus) - f0) / eps
        return J


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Build the Approximate Manifold Graph for UR5e upright constraint'
    )
    parser.add_argument('--n_c', type=int, default=1000,
                        help='Number of configurations (vertices) in the graph')
    parser.add_argument('--n_e', type=int, default=5,
                        help='Max edges per vertex (0 = no edges)')
    parser.add_argument('--output', type=str, default='/tmp/approx_graph.pkl',
                        help='Output file path for the serialized graph')
    parser.add_argument('--delta', type=float, default=0.01,
                        help='Inequality constraint threshold r_x^2, r_y^2 <= delta')
    parser.add_argument('--eps', type=float, default=0.05,
                        help='Diversity parameter epsilon (eq. 2.6)')
    parser.add_argument('--rho', type=float, default=0.05,
                        help='Diversity parameter rho (eq. 2.8)')
    parser.add_argument('--alpha_deg', type=float, default=30.0,
                        help='Diversity parameter alpha in degrees (eq. 2.7)')
    args = parser.parse_args()

    move_group = None
    if ROS_AVAILABLE:
        rospy.init_node('build_approx_graph', anonymous=True)
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            move_group = moveit_commander.MoveGroupCommander('manipulator')
            rospy.loginfo('MoveIt! move group initialized.')
        except Exception as e:
            rospy.logwarn(f'Could not initialize MoveIt! move group: {e}')
    else:
        print('[WARN] ROS not available – using placeholder FK.')

    constraint = UR5eUprightConstraint(move_group=move_group, delta=args.delta)
    # n=6 DOF, k=4 (6 - 2 constraint equations)
    graph = ApproximateManifoldGraph(
        constraint_fn=constraint.constraint_fn,
        jacobian_fn=constraint.jacobian_fn,
        config_dim=6,
        manifold_dim=4,   # k = n - m = 6 - 2 (two constraint equations)
        config_bounds=UR5E_JOINT_LIMITS,
        diversity_epsilon=args.eps,
        diversity_rho=args.rho,
        diversity_alpha=np.deg2rad(args.alpha_deg),
    )

    print(f'Building approximate manifold graph: n_c={args.n_c}, n_e={args.n_e}')
    graph.build(n_c=args.n_c, n_e=args.n_e)
    print(f'Graph built: {len(graph.vertices)} vertices, '
          f'{sum(len(v) for v in graph.adjacency.values()) // 2} edges')

    graph.save(args.output)
    print(f'Graph saved to {args.output}')


if __name__ == '__main__':
    main()
