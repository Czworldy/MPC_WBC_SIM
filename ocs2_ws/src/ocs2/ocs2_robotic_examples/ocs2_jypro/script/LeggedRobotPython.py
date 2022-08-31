import sys
import os
import rospkg
import numpy as np
sys.path.append(os.environ["HOME"]+"/MPC_WBC_sim/ocs2_ws/devel/lib/python3/dist-packages/ocs2_jypro")

from LeggedRobotPyBindings import mpc_interface
from LeggedRobotPyBindings import (
    scalar_array,
    vector_array,
    matrix_array,
    TargetTrajectories,
)

packageDir = rospkg.RosPack().get_path('ocs2_jypro')

taskFile = "mpc"
targetFile = os.path.join(packageDir, 'config/command/targetTrajectories.info')
urdfFile = os.path.join(rospkg.RosPack().get_path('JYPro'), 'urdf/JYPro_ocs2.urdf')
print(targetFile)
mpc = mpc_interface(taskFile, targetFile, urdfFile)
init_x = mpc.getInitState()
desiredTimeTraj = scalar_array()
desiredTimeTraj.push_back(3.0)

desiredInputTraj = vector_array()
initState = mpc.getInitState()
initState[7] = 0.5
print("initState: ", initState)

desiredInputTraj.push_back(np.zeros(mpc.getStateDim()))

desiredStateTraj = vector_array()
desiredStateTraj.push_back(initState)

targetTrajectories = TargetTrajectories(
    desiredTimeTraj, desiredStateTraj, desiredInputTraj
)
mpc.reset(targetTrajectories)
mpc.setModule("standing_trot")

dt = 0.02
t = 0
print(init_x)
for item in range(100):
    mpc.advanceMpc();
    t_result = scalar_array()
    x_result = vector_array()
    u_result = vector_array()

    mpc.getMpcSolution(t_result, x_result, u_result)
    
    dx = mpc.flowMap(t, x_result[0], u_result[0])
    x = x_result[0] + dx*dt
    t += dt
    mpc.setObservation(t, x, u_result[0])
    print(len(t_result))
    # print(t_result[0])
    # print(x_result[3])
    # print(u_result[0])