import rosbag

bag = rosbag.Bag('/home/MPC_WBC/dqwang/rapid_around.bag')
for (topic, msg, t) in bag.read_messages(topics=["/joint_states", "/imu"]):
    print(msg)
bag.close()