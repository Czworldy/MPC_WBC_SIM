import rospy
import numpy as np
from ocs2_msgs.msg import RegionForFoot, RegionSingle
from geometry_msgs.msg import Point
from Polygon import Polygon, Vertices

from pydrake.solvers import MathematicalProgram, Solve, GetAvailableSolvers, ProgramType, MixedIntegerBranchAndBound, MosekSolver
from pydrake.all import eq, le, ge

import matplotlib.pyplot as plt

import time

from enum import Enum

class LegId(Enum):
    LF_FOOT = 0
    RF_FOOT = 1
    LH_FOOT = 2
    RH_FOOT = 3

# Get the message from the topic
# Form a linear Constraint
# Create MIP Problems
# Solve the MIP Problems
# Create a new message

# foot_id use the same order in ocs2_jypro 
# contactNames3DoF{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"

WITHDRAW = 1
FOOT_X = 0.30164
FOOT_Y = 0.18322
M = 1e6
class PolygonReciver:
  def __init__(self):
    self.recived_polygons = [0, 1, 2, 3]
    self.start_callback = False
    # self.polygon_sub_lf = rospy.Subscriber('foothold_planner/RegionForFoot_LF', RegionForFoot, self.foothold_callback, queue_size=1)
    # self.polygon_sub_rf = rospy.Subscriber('foothold_planner/RegionForFoot_RF', RegionForFoot, self.foothold_callback, queue_size=1)
    # self.polygon_sub_lh = rospy.Subscriber('foothold_planner/RegionForFoot_LH', RegionForFoot, self.foothold_callback, queue_size=1)
    # self.polygon_sub_rh = rospy.Subscriber('foothold_planner/RegionForFoot_RH', RegionForFoot, self.foothold_callback, queue_size=1)
    print(self.recived_polygons)
    if WITHDRAW:
      self.fig = plt.figure()
      self.ax = plt.gca()
      self.ax.plot(0, 0, 'o')
    self.start_callback = True
    

  def foothold_callback(self, msg):
    if not self.start_callback:
      print("not start_callback")
      return
    
    foot_id = msg.foot_id
    recived_polygon_one_foot = []
    
    for region in msg.region:
      vertices = Vertices()
      
      for point in region.boundaryPoint:
        vertices.push_back([point.x, point.y])
        
        if WITHDRAW:
          self.ax.plot(point.x, point.y, '.', color = 'green')
          
      polygon = Polygon(vertices)
      recived_polygon_one_foot.append(polygon)  
      
    self.recived_polygons[foot_id] = recived_polygon_one_foot

    # if foot_id == 0:
    #   self._form_constraint_matrix()
    self._form_constraint_matrix()
  
  def _form_constraint_matrix(self):
    if (self.recived_polygons[0] == 0) or (self.recived_polygons[1] == 1) or (self.recived_polygons[2] == 2) or (self.recived_polygons[3] == 3):
      print("#############not recived all polygons#############")
      return

    time0 = time.time()
  
    prog = MathematicalProgram()
    
    #Add constraint
    for foot_id, one_foot_polygons in enumerate(self.recived_polygons):
      a = prog.NewBinaryVariables(len(one_foot_polygons), "a_%d"%(foot_id))
      p = prog.NewContinuousVariables(2, 1, "p_%d"%(foot_id))
      
      prog.AddConstraint(sum(a) == 1)
      
      for index, polygon in enumerate(one_foot_polygons):
        polygon.convert2InequalityConstraints()
        A = polygon.constraintA
        b = polygon.constraintb

        b = b.reshape((A.shape[0],1)) # very important to reshape the b

        prog.AddConstraint(le(A.dot(p) - b - (1-a[index])*M , np.zeros(shape=(A.shape[0],1))))
      if foot_id == LegId.LF_FOOT.value:
        prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([FOOT_X, FOOT_Y]), vars = p)
      elif foot_id == LegId.RF_FOOT.value:
        prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([FOOT_X, -FOOT_Y]), vars = p)
      elif foot_id == LegId.LH_FOOT.value:
        prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([-FOOT_X, FOOT_Y]), vars = p)
      elif foot_id == LegId.RH_FOOT.value:
        prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([-FOOT_X, -FOOT_Y]), vars = p)
      else:
        print(LegId.LF_FOOT)
        print(foot_id)
        raise ValueError("foot_id is not in the LegId")
    
    result = Solve(prog)   
    # print(prog.decision_variables())
    var = result.GetSolution()
    print(f"optimal solution p: {var}")
    if WITHDRAW:
      self.ax.plot(var[2], var[3], 'x', color = 'red')
      self.ax.plot(var[6], var[7], 'x', color = 'red')
      self.ax.plot(var[10], var[11], 'x', color = 'red')
      self.ax.plot(var[14], var[15], 'x', color = 'red')
      plt.axis('equal')
      self.ax.set_xlim(-1, 1)
      self.ax.set_ylim(-1, 1)
      plt.show()
   
    # print("Success? ", result.is_success())
    # print("result:", result.get_solution_result())
    # print("result: get_solver_details", result.get_solver_details().rescode, result.get_solver_details().solution_status)
    # print('solver is: ', result.get_solver_id().name())
    print("result: time ", result.get_solver_details().optimizer_time)
    usingtime = time.time() - time0
    print(usingtime)
    # print(f"optimal solution a: {result.GetSolution(a)}")
    
    
if __name__ == '__main__':
  rospy.init_node('foothold_listener')
  polygon_reciver = PolygonReciver()
  
  # debug solver one times
  msg1 = rospy.wait_for_message('foothold_planner/RegionForFoot_LF', RegionForFoot)
  msg2 = rospy.wait_for_message('foothold_planner/RegionForFoot_RF', RegionForFoot)
  msg3 = rospy.wait_for_message('foothold_planner/RegionForFoot_LH', RegionForFoot)
  msg4 = rospy.wait_for_message('foothold_planner/RegionForFoot_RH', RegionForFoot)
  
  polygon_reciver.foothold_callback(msg1)
  polygon_reciver.foothold_callback(msg2)
  polygon_reciver.foothold_callback(msg3)
  polygon_reciver.foothold_callback(msg4)
  
  # polygon_reciver._form_constraint_matrix()
  

  # while not rospy.is_shutdown():
  #   rospy.spin()
  #   print(polygon_reciver.recived_polygons)
