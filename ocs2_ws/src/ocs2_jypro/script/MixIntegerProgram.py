from pydrake.solvers import MathematicalProgram, Solve, GetAvailableSolvers, ProgramType, MixedIntegerBranchAndBound, MosekSolver
from pydrake.all import eq, le, ge
import numpy as np
import math
import time

prog = MathematicalProgram()

for sol in GetAvailableSolvers(ProgramType.kMIQP):
  print(sol.name())
  
# mosek = MosekSolver()

a = prog.NewBinaryVariables(2, 'a_1')
p = prog.NewContinuousVariables(2, 1, 'p')
M = 1e9

prog.AddConstraint(sum(a)  == 1)
A1 = np.matrix([[1, 1], [0, 1], [-1, 0], [0, -1]])
b1 = np.array([[0], [0], [-1], [-1]]) # A*p >= b   b means lower bound

A3 = np.matrix([[ 4.89918947, 0.40991068],
 [ 3.24556759 , 3.2611261 ],
 [ 0.39573772 , 4.89876913],
 [-0.41455069 , 4.89743   ],
 [-3.26111944 , 3.24555969],
 [-4.90049437 , 0.39110446],
 [-4.89948317 ,-0.4091349 ],
 [-3.24470584 ,-3.26030078],
 [-0.3856894  ,-4.90254573],
 [ 0.40457238 ,-4.9012425 ],
 [ 3.26030887 ,-3.24471288],
 [ 4.90082026 ,-0.39024633],])

b3 = np.array([ 2.78985004, 2.84631174, 2.21971468, 1.93837182, 0.58604037, -0.61330677,
 -0.78978049, -0.84583048, -0.21706393, 0.0573248,  1.41386561, 2.61360942])
b3 = b3.reshape((12,1))


A2 = np.matrix([[1, 0], [0, 1], [-1, 0], [0, -1]])
b2 = np.array([[1], [0], [-2], [-1]]) # A*p >= b   b means lower bound
print(A2.shape[0])
prog.AddConstraint(ge((A1 * p).A - b1 + (1-a[0])*M , np.zeros(shape=(4,1))))
prog.AddConstraint(ge((A2 * p).A - b2 + (1-a[1])*M , np.zeros(shape=(4,1))))
# prog.AddLinearConstraint(p[0] + 4 *p[1] <= 10)
# prog.AddConstraint(func, lb, ub, vars)

# def constraint1(p):
#   A1 = np.matrix([[1, 0], [0, 1], [-1, 0], [0, -1]])
#   b1 = np.matrix([[0], [0], [-1], [-1]]) # A*p >= b   b means lower bound  
#   return A1 * p - b1
# x = np.matrix([[-0.5],[-0.5]])
# print(constraint1(x))
# prog.AddConstraint(constraint1, lb=np.array([[0], [0], [0], [0]]), ub=np.array([[100], [100], [100], [100]]), vars=p)
# print(prog.decision_variable(0))

prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([3, 0.5]), vars = p)
solver = MosekSolver()
time0 = time.time()
result = solver.Solve(prog, initial_guess = np.array([0, 1, 1.5, 0.5]))
usingtime = time.time() - time0
# print(usingtime)
# print(f"optimal solution p: {result.GetSolution(p)}")
# print(f"optimal solution a: {result.GetSolution(a)}")
# print(f"optimal cost: {result.get_optimal_cost()}")

# print("Success? ", result.is_success())
# print("result:", result.get_solution_result())
# print("result: get_solver_details", result.get_solver_details().rescode, result.get_solver_details().solution_status)
# print("result: time ", result.get_solver_details().optimizer_time)
# print('solver is: ', result.get_solver_id().name())

# print(prog)

new_prog = MathematicalProgram()
p = new_prog.NewContinuousVariables(2, 1, 'p')
new_prog.AddConstraint(le(A3.dot(p) - b3 , np.zeros(shape=(12,1))))
new_prog.AddQuadraticErrorCost(Q = np.eye(2), x_desired = np.array([0, 0]), vars = p)
result = solver.Solve(new_prog)
print(f"optimal solution p: {result.GetSolution(p)}")
print("Success? ", result.is_success())
print("result:", result.get_solution_result())
print("result: get_solver_details", result.get_solver_details().rescode, result.get_solver_details().solution_status)
print("result: time ", result.get_solver_details().optimizer_time)
print('solver is: ', result.get_solver_id().name())
print(new_prog)



# solver = MixedIntegerBranchAndBound(prog, NloptSolver().solver_id())

# print(f"optimal solution p: {solver.GetSolution(p)}")
# print(f"optimal solution a: {solver.GetSolution(a)}")
# print(f"optimal cost: {solver.get_optimal_cost()}")
