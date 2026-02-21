#!/usr/bin/env python3
"""
scene_loader.py

Load a YAML scene file describing 3-D obstacles and provide:
  1. A standalone geometric collision checker (UR5e FK + link capsules vs obstacles).
     Works without ROS.
  2. A helper that populates the MoveIt! PlanningSceneInterface from the same file.
     Used inside the ROS planning node.

Scene file format (YAML)
------------------------
::

    name: "sim_scene_1"
    description: "Optional description"
    obstacles:
      - name: "table"
        type: box                  # box | sphere | cylinder
        size: [1.2, 0.8, 0.05]    # [x, y, z] extents in metres  (box only)
        pose:
          xyz: [0.5, 0.0, 0.35]   # [x, y, z] in metres
          rpy: [0.0, 0.0, 0.0]    # [roll, pitch, yaw] in radians
      - name: "column"
        type: cylinder
        radius: 0.05              # metres  (cylinder / sphere)
        height: 0.3               # metres  (cylinder only)
        pose:
          xyz: [0.6, 0.1, 0.55]
          rpy: [0.0, 0.0, 0.0]
      - name: "ball_valve"
        type: sphere
        radius: 0.03
        pose:
          xyz: [0.3, -0.1, 0.6]
          rpy: [0.0, 0.0, 0.0]

Supported shape types
---------------------
* ``box``      – axis-aligned bounding box (after pose rotation)
* ``sphere``   – sphere
* ``cylinder`` – upright cylinder (Z-axis, after pose rotation)
"""

import math
import os
import numpy as np
import yaml


# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

class SceneObject:
    """A single static obstacle in the scene."""

    def __init__(self, name, obj_type, pose_xyz, pose_rpy, **geometry):
        """
        Parameters
        ----------
        name : str
        obj_type : str  – 'box', 'sphere', or 'cylinder'
        pose_xyz : array-like of shape (3,)  – position in world frame [m]
        pose_rpy : array-like of shape (3,)  – roll/pitch/yaw [rad]
        **geometry : shape-specific keyword arguments
            box      → size=[sx, sy, sz]
            sphere   → radius=r
            cylinder → radius=r, height=h
        """
        self.name = name
        self.type = obj_type
        self.position = np.array(pose_xyz, dtype=float)
        self.rpy = np.array(pose_rpy, dtype=float)
        self.rotation = _rpy_to_matrix(*pose_rpy)
        self.geometry = geometry

    def __repr__(self):
        return f"SceneObject(name={self.name!r}, type={self.type!r})"


# ---------------------------------------------------------------------------
# Scene file I/O
# ---------------------------------------------------------------------------

class SceneLoader:
    """Read a YAML scene file and return a list of SceneObject instances."""

    @staticmethod
    def from_yaml(path):
        """
        Load obstacles from a YAML scene file.

        Parameters
        ----------
        path : str
            Absolute or relative path to the YAML file.

        Returns
        -------
        objects : list of SceneObject
        meta : dict
            Top-level metadata from the file (name, description, …).
        """
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        obstacles_raw = data.get('obstacles', [])
        objects = []
        for raw in obstacles_raw:
            obj = SceneLoader._parse_obstacle(raw)
            objects.append(obj)

        meta = {k: v for k, v in data.items() if k != 'obstacles'}
        return objects, meta

    @staticmethod
    def _parse_obstacle(raw):
        name = raw.get('name', 'unnamed')
        obj_type = raw.get('type', 'box').lower()
        pose = raw.get('pose', {})
        xyz = pose.get('xyz', [0.0, 0.0, 0.0])
        rpy = pose.get('rpy', [0.0, 0.0, 0.0])

        if obj_type == 'box':
            size = raw.get('size', [0.1, 0.1, 0.1])
            return SceneObject(name, 'box', xyz, rpy, size=np.array(size, dtype=float))

        if obj_type == 'sphere':
            radius = float(raw.get('radius', 0.05))
            return SceneObject(name, 'sphere', xyz, rpy, radius=radius)

        if obj_type == 'cylinder':
            radius = float(raw.get('radius', 0.05))
            height = float(raw.get('height', 0.2))
            return SceneObject(name, 'cylinder', xyz, rpy, radius=radius, height=height)

        raise ValueError(f"Unknown obstacle type {obj_type!r} for obstacle {name!r}. "
                         "Supported: box, sphere, cylinder.")

    # ------------------------------------------------------------------
    # MoveIt! integration (requires rospy + moveit_commander)
    # ------------------------------------------------------------------

    @staticmethod
    def load_into_moveit(path, scene_interface, frame_id='world'):
        """
        Populate a MoveIt! PlanningSceneInterface from a YAML scene file.

        Parameters
        ----------
        path : str
            Path to the YAML scene file.
        scene_interface : moveit_commander.PlanningSceneInterface
        frame_id : str
            TF frame for the scene (default 'world').
        """
        try:
            from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
            from std_msgs.msg import Header
            import tf.transformations as tft
        except ImportError as e:
            raise ImportError(
                "MoveIt! ROS packages not available. "
                "Use SceneCollisionChecker for standalone mode."
            ) from e

        objects, _ = SceneLoader.from_yaml(path)
        import rospy
        rospy.sleep(0.5)  # Give PlanningScene time to start
        scene_interface.clear()
        rospy.sleep(0.5)

        for obj in objects:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame_id
            q = tft.quaternion_from_euler(*obj.rpy)
            pose_stamped.pose.position.x = obj.position[0]
            pose_stamped.pose.position.y = obj.position[1]
            pose_stamped.pose.position.z = obj.position[2]
            pose_stamped.pose.orientation.x = q[0]
            pose_stamped.pose.orientation.y = q[1]
            pose_stamped.pose.orientation.z = q[2]
            pose_stamped.pose.orientation.w = q[3]

            if obj.type == 'box':
                s = obj.geometry['size']
                scene_interface.add_box(obj.name, pose_stamped,
                                        size=(s[0], s[1], s[2]))
            elif obj.type == 'sphere':
                r = obj.geometry['radius']
                scene_interface.add_sphere(obj.name, pose_stamped, radius=r)
            elif obj.type == 'cylinder':
                r = obj.geometry['radius']
                h = obj.geometry['height']
                scene_interface.add_cylinder(obj.name, pose_stamped,
                                             height=h, radius=r)
        rospy.sleep(0.5)
        rospy.loginfo(f"Loaded {len(objects)} obstacle(s) from {path!r}")


# ---------------------------------------------------------------------------
# UR5e forward kinematics (DH parameters)
# ---------------------------------------------------------------------------

class UR5eFk:
    """
    Forward kinematics for the Universal Robots UR5e arm.

    Uses standard Denavit-Hartenberg (DH) parameters from the UR5e datasheet:

      Joint  |  a [m]   |  d [m]   |  alpha [rad]
      -------|----------|----------|---------------
        1    |  0.0000  |  0.1625  |  pi/2
        2    | -0.4253  |  0.0000  |  0
        3    | -0.3922  |  0.0000  |  0
        4    |  0.0000  |  0.1333  |  pi/2
        5    |  0.0000  |  0.0997  | -pi/2
        6    |  0.0000  |  0.0996  |  0
    """

    _A = [0.0, -0.4253, -0.3922, 0.0, 0.0, 0.0]
    _D = [0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996]
    _ALPHA = [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0]

    # Conservative link radii for collision checking [m].
    # These approximate the physical link dimensions.
    LINK_RADII = [0.07, 0.065, 0.06, 0.055, 0.045, 0.04]

    @classmethod
    def link_transforms(cls, joints):
        """
        Compute the 4x4 homogeneous transforms for each joint frame.

        Parameters
        ----------
        joints : array-like of length 6  (joint angles in radians)

        Returns
        -------
        frames : list of 7 np.ndarray of shape (4, 4)
            frames[0] = base frame (identity)
            frames[i] = transform of joint i frame w.r.t. world, i=1…6
        """
        T = np.eye(4)
        frames = [T.copy()]
        for i, (q, a, d, alpha) in enumerate(
                zip(joints, cls._A, cls._D, cls._ALPHA)):
            Ti = _dh_matrix(q, a, d, alpha)
            T = T @ Ti
            frames.append(T.copy())
        return frames

    @classmethod
    def joint_positions(cls, joints):
        """
        Return the 3-D world position of each joint origin.

        Parameters
        ----------
        joints : array-like of length 6

        Returns
        -------
        positions : list of 7 np.ndarray of shape (3,)
            positions[0] = base (world origin)
            positions[i] = joint i position for i=1…6
        """
        frames = cls.link_transforms(joints)
        return [T[:3, 3] for T in frames]


# ---------------------------------------------------------------------------
# Standalone geometric collision checker
# ---------------------------------------------------------------------------

class SceneCollisionChecker:
    """
    Geometry-based collision checker for the UR5e arm against scene obstacles.

    Works entirely in Python/NumPy—no ROS or MoveIt! required.

    The robot links are modelled as capsules (line segment + radius).  Each
    capsule is sampled at several points; any sample point that penetrates an
    obstacle counts as a collision.

    Parameters
    ----------
    objects : list of SceneObject
        Obstacles loaded with SceneLoader.from_yaml().
    link_samples : int
        Number of intermediate sample points along each link (default 5).
    """

    def __init__(self, objects, link_samples=5):
        self.objects = objects
        self.link_samples = link_samples

    def is_free(self, joints):
        """
        Return True if the arm configuration is collision-free.

        Parameters
        ----------
        joints : array-like of length 6  (joint angles in radians)
        """
        positions = UR5eFk.joint_positions(joints)
        radii = UR5eFk.LINK_RADII

        # Check each link (segment between consecutive joint origins)
        for i in range(len(positions) - 1):
            p_start = positions[i]
            p_end = positions[i + 1]
            r = radii[i]

            # Sample along the link
            ts = np.linspace(0.0, 1.0, self.link_samples + 2)
            for t in ts:
                p = (1.0 - t) * p_start + t * p_end
                for obj in self.objects:
                    if _point_collides(p, r, obj):
                        return False
        return True


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _dh_matrix(theta, a, d, alpha):
    """Standard DH 4x4 transform matrix."""
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0],
    ])


def _rpy_to_matrix(roll, pitch, yaw):
    """Roll-Pitch-Yaw to 3x3 rotation matrix (extrinsic XYZ)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _point_collides(point, point_radius, obj):
    """
    Return True if a sphere of given radius centred at *point* intersects *obj*.

    Parameters
    ----------
    point : np.ndarray of shape (3,)
    point_radius : float
    obj : SceneObject
    """
    # Transform point into the object's local frame
    local = obj.rotation.T @ (point - obj.position)

    if obj.type == 'sphere':
        r_obj = obj.geometry['radius']
        return float(np.linalg.norm(local)) < r_obj + point_radius

    if obj.type == 'box':
        size = obj.geometry['size']
        half = size / 2.0
        # Closest point on box surface to *local*
        closest = np.clip(local, -half, half)
        dist = float(np.linalg.norm(local - closest))
        # dist < tolerance means the point is inside (or on the surface of) the box
        inside = dist < 1e-10 and bool(np.all(np.abs(local) <= half + 1e-10))
        return inside or dist < point_radius

    if obj.type == 'cylinder':
        r_obj = obj.geometry['radius']
        h = obj.geometry['height']
        # Cylinder axis is Z; check radial and axial distances
        radial_dist = float(math.hypot(local[0], local[1]))
        axial_dist = abs(local[2]) - h / 2.0
        # Point is inside the cylinder when both conditions hold
        inside = radial_dist <= r_obj and axial_dist <= 0
        if inside:
            return True
        # Closest point on cylinder surface to local
        if axial_dist <= 0:
            # Point is axially inside the cylinder extent
            dist = max(0.0, radial_dist - r_obj)
        else:
            # Point is above/below the cylinder – check against rim
            dist = math.hypot(max(0.0, radial_dist - r_obj), axial_dist)
        return dist < point_radius

    return False  # Unknown type – treat as free
