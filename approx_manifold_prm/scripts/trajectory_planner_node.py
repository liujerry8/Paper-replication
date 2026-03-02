#!/usr/bin/env python3
"""
trajectory_planner_node.py

ROS node: online LazyPRM trajectory planner (Section 2.2.5).

This node:
  1. Loads the pre-built approximate manifold graph G and offline roadmap M
     from disk at startup.
  2. Reconstructs the current scene via a NeRF-SLAM service (or loads a
     pre-built octomap from a topic).
  3. Exposes a ROS service ~/plan_trajectory that accepts a start and goal
     joint configuration and returns a MoveIt! RobotTrajectory.
  4. Post-processes the raw joint-space path using MoveIt!'s Iterative
     Parabolic Time Parameterization (IPTP) to produce velocity and
     acceleration profiles that satisfy the UR5e kinematic limits.

Subscribed topics
-----------------
  /joint_states  (sensor_msgs/JointState)  – current robot state

Published topics
----------------
  ~/planned_path  (moveit_msgs/DisplayTrajectory)  – for RViz visualization

Services
--------
  ~/plan_trajectory  (approx_manifold_prm/PlanTrajectory)  – trigger planning

Parameters
----------
  ~graph_file   (str)  Path to the serialized approximate graph (.pkl)
  ~roadmap_file (str)  Path to the serialized PRM* roadmap (.pkl)
  ~planning_group (str)  MoveIt! planning group name (default: 'manipulator')
  ~max_velocity_scaling    (float, 0-1, default: 1.0)
  ~max_acceleration_scaling (float, 0-1, default: 1.0)
"""

import os
import sys
import pickle
import numpy as np

try:
    import rospy
    import moveit_commander
    import moveit_msgs.msg as moveit_msgs
    from sensor_msgs.msg import JointState
    from moveit_commander import MoveGroupCommander, PlanningSceneInterface
    from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
    from moveit_msgs.srv import GetMotionPlan, GetMotionPlanRequest
    from approx_manifold_prm.srv import PlanTrajectory, PlanTrajectoryResponse
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from approx_manifold_graph import ApproximateManifoldGraph
from prm_star import PRMStar
from lazy_prm import LazyPRM
from build_approx_graph import UR5eUprightConstraint, UR5E_JOINT_LIMITS


# ---------------------------------------------------------------------------
# Collision helpers (wrapping MoveIt! planning scene)
# ---------------------------------------------------------------------------

def make_collision_checker(move_group):
    """Return a function that checks a joint config for collision."""
    from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
    from moveit_msgs.msg import RobotState
    from sensor_msgs.msg import JointState as JointStateMsg

    sv_srv = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
    sv_srv.wait_for_service(timeout=5.0)
    joint_names = move_group.get_active_joints()
    group_name = move_group.get_name()

    def check(joints):
        rs = RobotState()
        rs.joint_state = JointStateMsg()
        rs.joint_state.name = joint_names
        rs.joint_state.position = list(joints)

        req = GetStateValidityRequest()
        req.robot_state = rs
        req.group_name = group_name

        try:
            resp = sv_srv(req)
            return resp.valid
        except rospy.ServiceException:
            return False

    return check


def make_edge_checker(collision_checker, n_steps=10):
    """Return a function that checks a straight-line segment for collision."""
    def check(x_a, x_b):
        for t in np.linspace(0, 1, n_steps + 2)[1:-1]:
            x = (1 - t) * x_a + t * x_b
            if not collision_checker(x):
                return False
        return True
    return check


# ---------------------------------------------------------------------------
# Trajectory post-processing (IPTP via MoveIt!)
# ---------------------------------------------------------------------------

def time_parameterize(move_group, joint_path,
                      vel_scale=1.0, acc_scale=1.0):
    """
    Apply MoveIt!'s Iterative Parabolic Time Parameterization to a joint path.

    Parameters
    ----------
    move_group : MoveGroupCommander
    joint_path : list of list[float]
        Sequence of joint configurations.
    vel_scale, acc_scale : float
        Velocity and acceleration scaling factors (0, 1].

    Returns
    -------
    trajectory : RobotTrajectory | None
    """
    if not ROS_AVAILABLE:
        return None
    from moveit_commander.conversions import pose_to_list
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

    robot = moveit_commander.RobotCommander()
    joint_names = move_group.get_active_joints()

    jt = JointTrajectory()
    jt.joint_names = joint_names
    for cfg in joint_path:
        pt = JointTrajectoryPoint()
        pt.positions = list(cfg)
        jt.points.append(pt)

    rt = RobotTrajectory()
    rt.joint_trajectory = jt

    # Use MoveIt!'s time parameterization service if available
    try:
        from moveit_msgs.srv import (
            GetCartesianPath,
            ApplyPlanningScene,
        )
        move_group.set_max_velocity_scaling_factor(vel_scale)
        move_group.set_max_acceleration_scaling_factor(acc_scale)
        traj = move_group.retime_trajectory(
            robot.get_current_state(), rt, vel_scale, acc_scale
        )
        return traj
    except Exception as e:
        rospy.logwarn(f'Time parameterization failed: {e}')
        return rt


# ---------------------------------------------------------------------------
# ROS node
# ---------------------------------------------------------------------------

class TrajectoryPlannerNode:
    """Main ROS node for online LazyPRM trajectory planning."""

    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError('ROS is not available in this environment.')

        rospy.init_node('trajectory_planner', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        self.ready = False
        self.init_error = ''
        self.lazy_planner = None
        self.move_group = None
        self.vel_scale = rospy.get_param('~max_velocity_scaling', 1.0)
        self.acc_scale = rospy.get_param('~max_acceleration_scaling', 1.0)

        # Register the service early so it is always discoverable
        self.display_pub = rospy.Publisher(
            '~/planned_path', DisplayTrajectory, queue_size=1
        )
        self.srv = rospy.Service('~plan_trajectory', PlanTrajectory, self._plan_cb)

        try:
            self._initialize_planner()
            self.ready = True
            rospy.loginfo('TrajectoryPlannerNode ready.')
        except Exception as e:
            self.init_error = str(e)
            rospy.logerr(f'Planner initialization failed: {e}')
            rospy.logwarn('Service is registered but will return errors '
                          'until the planner is properly initialized.')

    def _initialize_planner(self):
        """Load graph, roadmap and build the LazyPRM planner."""
        graph_file = rospy.get_param('~graph_file', '/tmp/approx_graph.pkl')
        roadmap_file = rospy.get_param('~roadmap_file', '/tmp/roadmap.pkl')
        group_name = rospy.get_param('~planning_group', 'manipulator')
        scene_file = rospy.get_param('~scene_file', '')

        # MoveIt! setup
        self.move_group = MoveGroupCommander(group_name)
        self.scene = PlanningSceneInterface()

        # Load scene from file (if provided)
        if scene_file:
            from scene_loader import SceneLoader
            SceneLoader.load_into_moveit(scene_file, self.scene)
            rospy.loginfo(f'Scene loaded from {scene_file!r}')

        # Load approximate manifold graph
        constraint = UR5eUprightConstraint(move_group=self.move_group)
        self.graph = ApproximateManifoldGraph(
            constraint_fn=constraint.constraint_fn,
            jacobian_fn=constraint.jacobian_fn,
            config_dim=6,
            manifold_dim=4,   # k = n - m = 6 - 2 (two constraint equations)
            config_bounds=UR5E_JOINT_LIMITS,
        )
        self.graph.load(graph_file)
        rospy.loginfo(f'Loaded approx graph: {len(self.graph.vertices)} vertices')

        # Load roadmap
        with open(roadmap_file, 'rb') as f:
            roadmap = pickle.load(f)
        rospy.loginfo(f'Loaded roadmap: {roadmap.number_of_nodes()} nodes')

        # Build LazyPRM planner
        is_free = make_collision_checker(self.move_group)
        self.lazy_planner = LazyPRM(
            roadmap=roadmap,
            collision_checker=is_free,
            edge_checker=make_edge_checker(is_free),
            connect_k=rospy.get_param('~connect_k', 10),
        )

    def _plan_cb(self, req):
        """Handle a PlanTrajectory service request."""
        resp = PlanTrajectoryResponse()

        if not self.ready:
            resp.success = False
            resp.message = f'Planner not initialized: {self.init_error}'
            return resp

        x_start = np.array(req.start_joints)
        x_goal = np.array(req.goal_joints)

        time_limit = req.time_limit if req.time_limit > 0 else rospy.get_param('~time_limit', 10.0)

        rospy.loginfo(f'Planning from {x_start} to {x_goal}')

        try:
            path, planning_time_ms = self.lazy_planner.plan(
                x_start, x_goal, time_limit=time_limit
            )
        except Exception as e:
            rospy.logerr(f'Planning failed with exception: {e}')
            resp.success = False
            resp.planning_time_ms = 0.0
            resp.message = f'Planning exception: {e}'
            return resp

        resp.planning_time_ms = planning_time_ms

        if path is None:
            rospy.logwarn('LazyPRM failed to find a path.')
            resp.success = False
            resp.message = 'LazyPRM failed to find a path.'
            return resp

        rospy.loginfo(f'Path found ({len(path)} waypoints, {planning_time_ms:.1f} ms)')

        traj = time_parameterize(
            self.move_group, path, self.vel_scale, self.acc_scale
        )
        if traj is not None:
            resp.trajectory = traj
            resp.success = True
            resp.message = f'Path found: {len(path)} waypoints in {planning_time_ms:.1f} ms'
            # Publish for RViz
            display = DisplayTrajectory()
            display.trajectory.append(traj)
            self.display_pub.publish(display)
        else:
            resp.success = False
            resp.message = 'Time parameterization failed.'

        return resp

    def spin(self):
        rospy.spin()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    if not ROS_AVAILABLE:
        print('[ERROR] This node requires ROS and MoveIt! to run.')
        sys.exit(1)
    node = TrajectoryPlannerNode()
    node.spin()
