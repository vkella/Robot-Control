import unittest
import numpy as np
from Task_1 import *

class Testcode(unittest.TestCase):

    def setUp(self):
        # Create a sample point cloud for testing
        self.row_np_array = np.load("4.npz")
        self.row_pointcloud = self.row_np_array['arr_0']
        self.line_len=4
        self.line_angle=-86.85

    # Checking if function retuns x, y, z
    def test_read_pointcloud(self):
        
        pointcloud = read_pointcloud("4.npz")
        self.assertIsInstance(pointcloud, np.ndarray)
        self.assertEqual(pointcloud.shape[1], 3)
        
   # checking if the detected line returns 2 poins in array format
    def test_line_detection(self):
        dominant_line = line_detection(self.row_pointcloud)
        self.assertIsInstance(dominant_line, tuple)
        self.assertEqual(len(dominant_line), self.line_len)  
# Checking the angle claulation with known data
    def test_calculate_angle(self):
        lines = (19,1002,129,-994)
        angle = calculate_angle(lines)
        self.assertIsInstance(angle, float)
        self.assertEqual(round(angle,2),self.line_angle) 


if __name__ == '__main__':
    unittest.main()
