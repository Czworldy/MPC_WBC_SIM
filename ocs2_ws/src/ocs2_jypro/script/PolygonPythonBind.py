import rospy
import numpy as np
# from ocs2_msgs.msg import RegionForFoot, RegionSingle
# from geometry_msgs.msg import Point
# import os
# import sys
# sys.path.append(os.path.join(os.path.dirname(__file__)))
from Polygon import Polygon, Vertices

pos = Vertices()
pos.push_back([1, 1])
pos.push_back([0, 1])
pos.push_back([0, 0])
pos.push_back([1, 0])
poly = Polygon(pos)
print(poly.getVertex(0))
print(poly.getArea())
# A = np.zeros(2,dtype=np.float64)
# b = np.zeros(2,dtype=np.float64)
poly.convert2InequalityConstraints()
A = poly.constraintA
b = poly.constraintb
print(A)
print(b)