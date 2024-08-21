import math
import time
import rospy
import pickle
import numpy as np
from nav_msgs.msg import Path
from ocs2_msgs.msg import mpc_observation, mpc_target_trajectories, mpc_state, mpc_input



start_time = None
def mpc_observation_callback(msg):
    global start_time
    if start_time is None:
        start_time = msg.time 

def main():
    global start_time
    rate = rospy.Rate(10) 
    rospy.Subscriber("/humanoid_mpc_observation", mpc_observation, mpc_observation_callback)
    target_pub = rospy.Publisher("/humanoid_mpc_target", mpc_target_trajectories, queue_size=10)
    
    path = './path_2.pickle'
    with open(path, 'rb') as file:
        params_dict = pickle.load(file)
        print(params_dict)
    
    time.sleep(1)
    
    publish_done = False
    target_path = mpc_target_trajectories()
    while not rospy.is_shutdown():
        if start_time is not None and not publish_done:
            
            time_traj = np.array(params_dict['timeTrajectory']) + start_time
            
            target_path.timeTrajectory = []
            target_path.stateTrajectory = []
            target_path.inputTrajectory = []
            
            for i in range(0, len(time_traj), 2):
                target_path.timeTrajectory.append(time_traj[i])
                
                tmp_state = mpc_state()
                tmp_state.value = params_dict['stateTrajectory'][i]
                
                tmp_input = mpc_input()
                tmp_input.value = params_dict['inputTrajectory'][i]
                tmp_input.value[5] = 1.36 
                
                target_path.stateTrajectory.append(tmp_state)
                target_path.inputTrajectory.append(tmp_state)
            
            target_pub.publish(target_path)
            print("published")
            publish_done = True
        
        rate.sleep()
          
          
def quaternion_to_euler(orientation, target='roll'):
    '''
    Convert a quaternion into euler angles (roll, pitch, yaw)
    '''
    x, y, z, w = orientation
    if target == 'roll':
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        return roll
    elif target == 'pitch':
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        return pitch
    elif target == 'yaw':
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw
    elif target == 'all':
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw
    else:
        raise ValueError('Invalid target: {}'.format(target))
          

timeTrajectory = None
stateTrajectory = None
inputTrajectory = None

def regular_yaw(x, ref):
    ub = ref + math.pi
    lb = ref - math.pi
    if(x > ub):
        x = lb + (x - lb) % (2 * math.pi)
    else:
        x = ub - (ub - x) % (2 * math.pi)
    return x
    
def path_callback(msg):
    global timeTrajectory, stateTrajectory, inputTrajectory
    poses = msg.poses
    
    curr_stamp = None
    last_yaw = None
    start_stamp = poses[0].header.stamp.secs
    timeTrajectory, stateTrajectory, inputTrajectory = [], [], []
    for pose in poses:
        time_stamp = pose.header.stamp.secs - start_stamp
        if time_stamp != curr_stamp:
            x = pose.pose.position.x
            y = pose.pose.position.y
            ori = pose.pose.orientation
            yaw = quaternion_to_euler((ori.x, ori.y, ori.z, ori.w), target="yaw")
            if last_yaw is None:
                last_yaw = yaw
            yaw = regular_yaw(yaw, last_yaw)
            timeTrajectory.append(time_stamp)
            stateTrajectory.append([0, 0, yaw, x, y, 1.36,
                                    0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 0, 0, ])
            inputTrajectory.append([0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0, 
                                    0, 0, 0, 0, 0])
            curr_stamp = time_stamp
    
    print("pose_length : ", len(timeTrajectory))
            
        
        
        
def main_path():
    rospy.Subscriber("/ov_msckf/pathimu", Path, path_callback)
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        save_path = './path_2.pickle'
        with open(save_path, 'wb') as file:
            params_dict = {
                'timeTrajectory': timeTrajectory,
                'stateTrajectory': stateTrajectory,
                'inputTrajectory': inputTrajectory
                }
            pickle.dump(params_dict, file)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        rospy.init_node('path_transform', anonymous=True)
        main()
        # main_path()
        
        # read
        # path = './path_2.pickle'
        # with open(path, 'rb') as file:
        #     params_dict = pickle.load(file)
        #     print(params_dict)
            
            
    except rospy.ROSInterruptException:
        pass
