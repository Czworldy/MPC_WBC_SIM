from pydrake.solvers import MathematicalProgram, Solve
import numpy as np
import math


prog = MathematicalProgram()
polygon_oder = 7
polygon_coffe_num = polygon_oder + 1
d_order = 3
dT = 0.175
x = prog.NewContinuousVariables(2 * polygon_coffe_num, "x")

Q = np.zeros((2 * polygon_coffe_num, 2 * polygon_coffe_num))

for k in range(2):
  for i in range(d_order, polygon_coffe_num):
    for j in range(d_order, polygon_coffe_num):
      Q[k*polygon_coffe_num + i,k*polygon_coffe_num + j] = 1.0*math.factorial(i) / math.factorial(i - d_order) * math.factorial(j) / math.factorial(j - d_order) / (i + j - 2 * d_order + 1) #/ (dT ** (6))  
prog.AddQuadraticCost(Q = Q, b=np.zeros((2 * polygon_coffe_num)), vars=x)

print(Q)


Aeq_block_1 =  np.zeros((6 , polygon_coffe_num))
Aeq_block_2 =  np.zeros((6 , polygon_coffe_num))
Aeq_block_1[0,0] = 1
Aeq_block_1[1,1] = 1
Aeq_block_1[2,2] = 2
Aeq_block_1[3,:] = np.array([0.5**x for x in range(polygon_coffe_num)])
Aeq_block_1[4,:] = np.array([1 for x in range(polygon_coffe_num)])
Aeq_block_1[5,:] = np.array([x for x in range(polygon_coffe_num)])

Aeq_block_2[0,0] = 1
Aeq_block_2[1,1] = 1
Aeq_block_2[2,:] = np.array([0,0,2,6,12,20,30,42])
Aeq_block_2[3,:] = np.array([0.5**x for x in range(polygon_coffe_num)])
Aeq_block_2[4,:] = np.array([1 for x in range(polygon_coffe_num)])
Aeq_block_2[5,:] = np.array([x for x in range(polygon_coffe_num)])
# print(Aeq_block_1)
Aeq = np.block([
  [Aeq_block_1 , np.zeros((6, polygon_coffe_num))],
  [np.zeros((6, polygon_coffe_num)), Aeq_block_2],
  [0,0,2,6,12,20,30,42,0,0,-2,0,0,0,0,0],
])
print(Aeq)
# p1 v1 a1 p2 p3 v3 p3 v3 a5 p4 p5 v5 0
# 0, 0.5,1,0.1,0.15,0,0.15,0,0,0.1,0,0,0
beq = np.array([0,0.2,1,  0.12,  0.15,0, 0.15,0,  0,  0.1,  0,0,    0])
prog.AddLinearEqualityConstraint(Aeq=Aeq, beq=beq, vars=x)

result = Solve(prog)
print(f"optimal solution x: {result.GetSolution(x)}")
print(f"optimal cost: {result.get_optimal_cost()}")

C = result.GetSolution(x)

from pydrake.all import Polynomial_

Polynomial = Polynomial_[np.float64]
# traj = Polynomial([1,2,3,4])

traj1 = Polynomial(C[0:8])
traj2 = Polynomial(C[8:16])

print(C[8:16])
print(traj1.EvaluateUnivariate(0.5))
y = []
t = []
vel = [] 
acc = []
jerk = []
for i in range(101):
  y.append(traj1.EvaluateUnivariate(i/100.0))
  vel.append(traj1.EvaluateUnivariate(i/100.0, derivative_order = 1))
  acc.append(traj1.EvaluateUnivariate(i/100.0, derivative_order = 2))
  jerk.append(traj1.EvaluateUnivariate(i/100.0, derivative_order = 3))
  t.append(i/100.0)
for i in range(101):
  y.append(traj2.EvaluateUnivariate(i/100.0))
  vel.append(traj2.EvaluateUnivariate(i/100.0, derivative_order = 1))
  acc.append(traj2.EvaluateUnivariate(i/100.0, derivative_order = 2))
  jerk.append(traj2.EvaluateUnivariate(i/100.0, derivative_order = 3))
  t.append(1 + i/100.0)

import matplotlib.pyplot as plt

plt.plot(t,y, label = 'position')
plt.plot(t,vel, label = 'vel')
plt.plot(t,acc, label = 'acc')
# plt.plot(t,jerk, label = 'jerk')
plt.grid(True)

plt.show()


