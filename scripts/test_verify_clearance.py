#!/usr/bin/env python3
import unittest
import math
from unittest.mock import patch
import sys
import os

# Add current directory to path to import verify_clearance
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from verify_clearance import compute_lowest_z, get_euclidean_distance, get_gz_model_pose
from scipy.spatial.transform import Rotation as R

class TestVerifyClearance(unittest.TestCase):

    def setUp(self):
        self.radius = 0.015
        self.length = 0.3
        
    def test_compute_lowest_z_upright_on_floor(self):
        # Cylinder standing perfectly upright. Origin is at center (Z = 0.15)
        # Orientation is identity (no rotation)
        pos = [0.0, 0.0, 0.15]
        quat = [0.0, 0.0, 0.0, 1.0] 
        lowest = compute_lowest_z(pos, quat, self.radius, self.length)
        self.assertAlmostEqual(lowest, 0.0, places=4)
        
    def test_compute_lowest_z_knocked_over_on_floor(self):
        # Cylinder knocked over, rolling on the floor. Origin Z is equal to radius (0.015)
        # Rotated 90 degrees (pi/2) around X axis
        pos = [0.0, 0.0, 0.015]
        quat = R.from_euler('x', 90, degrees=True).as_quat()
        lowest = compute_lowest_z(pos, quat, self.radius, self.length)
        self.assertAlmostEqual(lowest, 0.0, places=4)

    def test_compute_lowest_z_tilted_on_floor(self):
        # Cylinder tilted at 45 degrees. 
        # The lowest Z relative to center is -(L/2)*cos(45) - R*sin(45)
        # = -0.15 * 0.7071 - 0.015 * 0.7071 = -0.106065 - 0.0106065 = -0.1166715
        # To be on the floor, its Z pos must be exactly +0.1166715
        pos = [0.0, 0.0, 0.1166715]
        quat = R.from_euler('x', 45, degrees=True).as_quat()
        lowest = compute_lowest_z(pos, quat, self.radius, self.length)
        self.assertAlmostEqual(lowest, 0.0, places=4)

    def test_compute_lowest_z_lifted_upright(self):
        # Picked up and held perfectly upright
        pos = [0.0, 0.0, 0.30] # lifted by 0.15m clearance
        quat = [0.0, 0.0, 0.0, 1.0]
        lowest = compute_lowest_z(pos, quat, self.radius, self.length)
        self.assertAlmostEqual(lowest, 0.15, places=4)
        
    def test_get_euclidean_distance(self):
        pos1 = [1.0, 2.0, 3.0]
        pos2 = [1.0, 2.0, 3.5]
        self.assertEqual(get_euclidean_distance(pos1, pos2), 0.5)
        
    @patch('subprocess.run')
    def test_get_gz_model_pose_parsing(self, mock_run):
        # Mock Gazebo output
        class MockResult:
            returncode = 0
            stdout = """
            pose {
              position {
                x: -1.87
                y: -2.0
                z: 0.15
              }
              orientation {
                x: 0.0
                y: 1.0
                z: 0.0
                w: 0.0
              }
            }
            """
        mock_run.return_value = MockResult()
        
        pos, quat = get_gz_model_pose("red_cylinder")
        self.assertEqual(pos, [-1.87, -2.0, 0.15])
        self.assertEqual(quat, [0.0, 1.0, 0.0, 0.0])

if __name__ == '__main__':
    unittest.main()
