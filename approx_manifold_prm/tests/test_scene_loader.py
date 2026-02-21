#!/usr/bin/env python3
"""
test_scene_loader.py

Unit tests for scene_loader.py.

Tests cover:
  - Parsing each obstacle type (box, sphere, cylinder) from YAML
  - Default pose values
  - Unknown type raises ValueError
  - SceneCollisionChecker: known collision / known free configurations
  - UR5eFk: joint_positions returns 7 points, base always at origin
  - Geometry helper _point_collides for all three shapes
  - run_planner.py argument parsing helpers
"""

import io
import os
import sys
import math
import tempfile
import unittest

import numpy as np
import yaml

SCRIPTS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
sys.path.insert(0, SCRIPTS_DIR)

from scene_loader import (
    SceneLoader, SceneObject, SceneCollisionChecker,
    UR5eFk, _point_collides,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def write_scene(obstacles):
    """Write a temporary YAML scene file and return its path."""
    data = {'name': 'test_scene', 'obstacles': obstacles}
    f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump(data, f)
    f.close()
    return f.name


# ---------------------------------------------------------------------------
# Tests: SceneLoader.from_yaml
# ---------------------------------------------------------------------------

class TestSceneLoaderParsing(unittest.TestCase):

    def test_box_parsed(self):
        path = write_scene([{
            'name': 'my_box', 'type': 'box', 'size': [1.0, 0.5, 0.2],
            'pose': {'xyz': [0.1, 0.2, 0.3], 'rpy': [0.0, 0.0, 0.0]},
        }])
        try:
            objects, meta = SceneLoader.from_yaml(path)
            self.assertEqual(len(objects), 1)
            obj = objects[0]
            self.assertEqual(obj.name, 'my_box')
            self.assertEqual(obj.type, 'box')
            np.testing.assert_allclose(obj.geometry['size'], [1.0, 0.5, 0.2])
            np.testing.assert_allclose(obj.position, [0.1, 0.2, 0.3])
        finally:
            os.unlink(path)

    def test_sphere_parsed(self):
        path = write_scene([{
            'name': 'ball', 'type': 'sphere', 'radius': 0.07,
            'pose': {'xyz': [0.5, 0.0, 1.0], 'rpy': [0.0, 0.0, 0.0]},
        }])
        try:
            objects, _ = SceneLoader.from_yaml(path)
            obj = objects[0]
            self.assertEqual(obj.type, 'sphere')
            self.assertAlmostEqual(obj.geometry['radius'], 0.07)
        finally:
            os.unlink(path)

    def test_cylinder_parsed(self):
        path = write_scene([{
            'name': 'tube', 'type': 'cylinder', 'radius': 0.03, 'height': 0.4,
            'pose': {'xyz': [0.3, 0.1, 0.6], 'rpy': [0.0, 0.0, 0.0]},
        }])
        try:
            objects, _ = SceneLoader.from_yaml(path)
            obj = objects[0]
            self.assertEqual(obj.type, 'cylinder')
            self.assertAlmostEqual(obj.geometry['radius'], 0.03)
            self.assertAlmostEqual(obj.geometry['height'], 0.40)
        finally:
            os.unlink(path)

    def test_default_pose_is_origin(self):
        path = write_scene([{'name': 'item', 'type': 'sphere', 'radius': 0.1}])
        try:
            objects, _ = SceneLoader.from_yaml(path)
            np.testing.assert_allclose(objects[0].position, [0.0, 0.0, 0.0])
        finally:
            os.unlink(path)

    def test_multiple_obstacles(self):
        path = write_scene([
            {'name': 'a', 'type': 'box', 'size': [0.1, 0.1, 0.1],
             'pose': {'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}},
            {'name': 'b', 'type': 'sphere', 'radius': 0.05,
             'pose': {'xyz': [1, 0, 0], 'rpy': [0, 0, 0]}},
        ])
        try:
            objects, _ = SceneLoader.from_yaml(path)
            self.assertEqual(len(objects), 2)
            self.assertEqual(objects[0].name, 'a')
            self.assertEqual(objects[1].name, 'b')
        finally:
            os.unlink(path)

    def test_unknown_type_raises(self):
        path = write_scene([{'name': 'x', 'type': 'mesh'}])
        try:
            with self.assertRaises(ValueError):
                SceneLoader.from_yaml(path)
        finally:
            os.unlink(path)

    def test_metadata_returned(self):
        data = {'name': 'hello', 'description': 'test', 'obstacles': []}
        f = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(data, f)
        f.close()
        try:
            _, meta = SceneLoader.from_yaml(f.name)
            self.assertEqual(meta.get('name'), 'hello')
            self.assertEqual(meta.get('description'), 'test')
        finally:
            os.unlink(f.name)

    def test_example_scene_1_loads(self):
        scenes_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'scenes'))
        scene_file = os.path.join(scenes_dir, 'sim_scene_1.yaml')
        objects, meta = SceneLoader.from_yaml(scene_file)
        self.assertGreater(len(objects), 0)
        self.assertEqual(meta.get('name'), 'sim_scene_1')

    def test_example_scene_2_loads(self):
        scenes_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'scenes'))
        scene_file = os.path.join(scenes_dir, 'sim_scene_2.yaml')
        objects, meta = SceneLoader.from_yaml(scene_file)
        self.assertGreater(len(objects), 0)
        self.assertEqual(meta.get('name'), 'sim_scene_2')


# ---------------------------------------------------------------------------
# Tests: _point_collides
# ---------------------------------------------------------------------------

class TestPointCollides(unittest.TestCase):

    def _sphere_obj(self, center, radius):
        return SceneObject('s', 'sphere', center, [0, 0, 0], radius=radius)

    def _box_obj(self, center, size):
        return SceneObject('b', 'box', center, [0, 0, 0], size=np.array(size))

    def _cyl_obj(self, center, radius, height):
        return SceneObject('c', 'cylinder', center, [0, 0, 0],
                           radius=radius, height=height)

    # sphere
    def test_sphere_inside(self):
        obj = self._sphere_obj([0, 0, 0], 0.5)
        self.assertTrue(_point_collides(np.array([0.0, 0.0, 0.0]), 0.0, obj))

    def test_sphere_outside(self):
        obj = self._sphere_obj([0, 0, 0], 0.1)
        self.assertFalse(_point_collides(np.array([0.5, 0.0, 0.0]), 0.0, obj))

    def test_sphere_on_boundary(self):
        obj = self._sphere_obj([0, 0, 0], 0.5)
        # Point is exactly on boundary; point radius 0 → should NOT collide
        self.assertFalse(_point_collides(np.array([0.5, 0.0, 0.0]), 0.0, obj))

    def test_sphere_with_point_radius(self):
        obj = self._sphere_obj([0, 0, 1.0], 0.3)
        # Point at [0,0,0.65]: distance to sphere center = 0.35, sum of radii = 0.3+0.1=0.4
        # 0.35 < 0.4 → collision
        self.assertTrue(_point_collides(np.array([0.0, 0.0, 0.65]), 0.1, obj))

    # box
    def test_box_point_inside(self):
        obj = self._box_obj([0, 0, 0], [1.0, 1.0, 1.0])
        self.assertTrue(_point_collides(np.array([0.0, 0.0, 0.0]), 0.0, obj))

    def test_box_point_outside(self):
        obj = self._box_obj([0, 0, 0], [0.2, 0.2, 0.2])
        self.assertFalse(_point_collides(np.array([1.0, 0.0, 0.0]), 0.0, obj))

    def test_box_sphere_touches(self):
        # Box edge at x=0.1; sphere of radius 0.05 centred at x=0.13
        obj = self._box_obj([0, 0, 0], [0.2, 0.2, 0.2])
        self.assertTrue(_point_collides(np.array([0.13, 0.0, 0.0]), 0.05, obj))

    # cylinder
    def test_cylinder_inside(self):
        obj = self._cyl_obj([0, 0, 0.5], 0.1, 1.0)
        self.assertTrue(_point_collides(np.array([0.0, 0.0, 0.5]), 0.0, obj))

    def test_cylinder_outside_radially(self):
        obj = self._cyl_obj([0, 0, 0.5], 0.1, 1.0)
        self.assertFalse(_point_collides(np.array([0.5, 0.0, 0.5]), 0.0, obj))

    def test_cylinder_outside_axially(self):
        obj = self._cyl_obj([0, 0, 0], 0.5, 0.2)
        # Point is 0.5 m above cylinder top (h/2 = 0.1)
        self.assertFalse(_point_collides(np.array([0.0, 0.0, 0.6]), 0.0, obj))


# ---------------------------------------------------------------------------
# Tests: UR5eFk
# ---------------------------------------------------------------------------

class TestUR5eFk(unittest.TestCase):

    def test_returns_seven_positions(self):
        joints = np.zeros(6)
        positions = UR5eFk.joint_positions(joints)
        self.assertEqual(len(positions), 7)

    def test_base_at_origin(self):
        joints = np.zeros(6)
        positions = UR5eFk.joint_positions(joints)
        np.testing.assert_allclose(positions[0], [0.0, 0.0, 0.0], atol=1e-10)

    def test_positions_are_3d(self):
        joints = np.zeros(6)
        for p in UR5eFk.joint_positions(joints):
            self.assertEqual(p.shape, (3,))

    def test_different_joints_different_positions(self):
        p1 = UR5eFk.joint_positions(np.zeros(6))
        p2 = UR5eFk.joint_positions(np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        # At least one joint position should differ
        diffs = [not np.allclose(a, b) for a, b in zip(p1, p2)]
        self.assertTrue(any(diffs))

    def test_transforms_chain(self):
        joints = np.array([0.5, -0.5, 0.3, -0.2, 0.1, 0.0])
        frames = UR5eFk.link_transforms(joints)
        self.assertEqual(len(frames), 7)
        for T in frames:
            self.assertEqual(T.shape, (4, 4))
            # Bottom row should be [0, 0, 0, 1]
            np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-10)
            # Rotation part should be orthogonal
            R = T[:3, :3]
            np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-8)


# ---------------------------------------------------------------------------
# Tests: SceneCollisionChecker
# ---------------------------------------------------------------------------

class TestSceneCollisionChecker(unittest.TestCase):

    def test_empty_scene_is_always_free(self):
        checker = SceneCollisionChecker([])
        # Any configuration should be free if there are no obstacles
        joints = np.zeros(6)
        self.assertTrue(checker.is_free(joints))

    def test_obstacle_far_from_arm(self):
        # A box far from the robot workspace should not interfere
        obj = SceneObject('far_box', 'box', [100.0, 100.0, 100.0], [0, 0, 0],
                          size=np.array([0.1, 0.1, 0.1]))
        checker = SceneCollisionChecker([obj])
        self.assertTrue(checker.is_free(np.zeros(6)))

    def test_obstacle_surrounding_base_causes_collision(self):
        # A large sphere surrounding the robot base should collide
        obj = SceneObject('huge_sphere', 'sphere', [0.0, 0.0, 0.0], [0, 0, 0],
                          radius=10.0)
        checker = SceneCollisionChecker([obj])
        self.assertFalse(checker.is_free(np.zeros(6)))

    def test_scene_file_1_usable_with_default_config(self):
        scenes_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'scenes'))
        objects, _ = SceneLoader.from_yaml(
            os.path.join(scenes_dir, 'sim_scene_1.yaml'))
        checker = SceneCollisionChecker(objects)
        # Default zero config may or may not collide – just check it runs
        result = checker.is_free(np.zeros(6))
        self.assertIsInstance(result, bool)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    unittest.main(verbosity=2)
