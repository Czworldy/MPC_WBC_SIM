import sys
import os
import rospkg
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