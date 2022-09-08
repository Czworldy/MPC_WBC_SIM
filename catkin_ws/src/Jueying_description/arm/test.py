import pybullet as p

physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-9.8)
arm = p.loadURDF("/model/arm.urdf")
p.resetJointState(arm, 4, 0.5)

while 1:
    a = 1