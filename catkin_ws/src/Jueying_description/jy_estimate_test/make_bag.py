#!/usr/bin/env python
import rospy
import rosbag
import os
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import time
import csv

def euler_to_quaternion(roll, pitch, yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

if __name__ == "__main__":
    new_bag_name = "/home/MPC_WBC/dqwang/rapid_around.bag"
    new_bag = rosbag.Bag(new_bag_name,'w')
    imu = []
    joint_state = []
    twist_state = []
    torque_state = []

    file = open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_estimate_test/data_x20/JY-P1-003 2021-05-21 15-48-41.csv")
    reader = csv.reader(file)
    result = list(reader)
    i = 0
    for data in result:
        # read the imu data
        imu_msg_time = rospy.Time.from_sec(float(data[0]))
        # quaternion = euler_to_quaternion(float(data[1])/180*3.1415926, float(data[2])/180*3.1415926, float(data[3])/180*3.1415926)
        # imu_orientation = [quaternion[0], quaternion[1], quaternion[2],quaternion[3]]
        # imu_angular_velocity = [float(data[4]), float(data[5]), float(data[6])]
        # imu_linear_acceleration = [float(data[7]), float(data[8]), float(data[9])]

        #此处需要将imu坐标旋转90度
        quaternion = euler_to_quaternion(-float(data[2])/180*3.1415926, float(data[1])/180*3.1415926, float(data[3])/180*3.1415926)
        imu_orientation = [quaternion[0], quaternion[1], quaternion[2],quaternion[3]]
        imu_angular_velocity = [-float(data[5]), float(data[4]), float(data[6])]
        imu_linear_acceleration = [-float(data[8]), float(data[7]), float(data[9])]
        imu_seq = i
        tmp_data = []
        tmp_data.append(imu_seq) # 0
        tmp_data.append(imu_msg_time) # 1
        tmp_data.append(imu_orientation) # 2
        tmp_data.append(imu_angular_velocity) # 3
        tmp_data.append(imu_linear_acceleration) # 4
        imu.append(tmp_data) 

        # read the joint state
        joint_msg_time = rospy.Time.from_sec(float(data[0]))
        joint_position = [-float(data[10]), float(data[11]), float(data[12]), #FL
                          float(data[13]), float(data[14]), float(data[15]), #FH
                          -float(data[16]), float(data[17]), float(data[18]), #LH
                          float(data[19]), float(data[20]), float(data[21]), #RH
                          ]
        joint_seq  = i
        tmp_data = []
        tmp_data.append(joint_seq)
        tmp_data.append(joint_msg_time)
        tmp_data.append(joint_position)
        joint_state.append(tmp_data)

        # read the joint velocity
        joint_msg_time = rospy.Time.from_sec(float(data[0]))
        joint_velocity = [-float(data[22]), float(data[23]), float(data[24]), #FL
                          float(data[25]), float(data[26]), float(data[27]), #FH
                          -float(data[28]), float(data[29]), float(data[30]), #LH
                          float(data[31]), float(data[32]), float(data[33]), #RH
                          ]
        joint_seq  = i
        tmp_data = []
        tmp_data.append(joint_seq)
        tmp_data.append(joint_msg_time)
        tmp_data.append(joint_velocity)
        twist_state.append(tmp_data)

        # read the joint torque
        joint_msg_time = rospy.Time.from_sec(float(data[0]))
        joint_torque   = [-float(data[34]), float(data[35]), float(data[36]), #FL
                          float(data[37]), float(data[38]), float(data[39]), #FH
                          -float(data[40]), float(data[41]), float(data[42]), #LH
                          float(data[43]), float(data[44]), float(data[45]), #RH
                          ]
        joint_seq  = i
        tmp_data = []
        tmp_data.append(joint_seq)
        tmp_data.append(joint_msg_time)
        tmp_data.append(joint_torque)
        torque_state.append(tmp_data)
        i=i+1

    seq_list_joint_state = [test1[0] for test1 in joint_state]
    msg_time_list_joint_state = [test2[1] for test2 in joint_state]
    state_list_joint_state = [test4[2] for test4 in joint_state]
    twist_list_twist_state = [test5[2] for test5 in twist_state]
    torque_list_torque_state = [test6[2] for test6 in torque_state]

    # combine joint message to a single topic
    joint_name = [ "LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE","LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"]
    for i in range(min([len(joint_state),len(twist_state),len(torque_state)])):
        joint_msgs = JointState()
        now_seq = seq_list_joint_state[i]
        joint_msgs.name = joint_name
        joint_msgs.header.seq = now_seq
        joint_msgs.header.frame_id = ''
        joint_msgs.header.stamp = msg_time_list_joint_state[i]
        #joint_msgs.header.stamp.nsec = msg_time_list_joint_state[i].to_nsec()
        joint_msgs.position = state_list_joint_state[i]
        joint_msgs.velocity = twist_list_twist_state[i]
        joint_msgs.effort = torque_list_torque_state[i]
        msgs_time = msg_time_list_joint_state[i]
        new_bag.write("/joint_states",joint_msgs,t=msgs_time)

    # custom IMU message
    for i in range(len(imu)):
        imu_msgs = Imu()
        imu_msgs.header.frame_id = 'imu_link'
        imu_msgs.header.stamp = imu[i][1]
        imu_msgs.header.seq = imu[i][0]
        imu_msgs.orientation.x = imu[i][2][0]
        imu_msgs.orientation.y = imu[i][2][1]
        imu_msgs.orientation.z = imu[i][2][2]
        imu_msgs.orientation.w = imu[i][2][3]
        imu_msgs.angular_velocity.x = imu[i][3][0]
        imu_msgs.angular_velocity.y = imu[i][3][1]
        imu_msgs.angular_velocity.z = imu[i][3][2]
        imu_msgs.linear_acceleration.x = imu[i][4][0]
        imu_msgs.linear_acceleration.y = imu[i][4][1]
        imu_msgs.linear_acceleration.z = imu[i][4][2]
        msgs_time = imu[i][1]
        new_bag.write("/sensors/imu",imu_msgs,t=msgs_time)
    print("process done")
    new_bag.close()
       