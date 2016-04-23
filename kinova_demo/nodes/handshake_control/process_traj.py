#!/usr/bin/python
import rosbag

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

bag = rosbag.Bag('/home/ubuntu_ros/handshake_demo/ben-april-22-3.bag')

target = open("ben_traj.txt", 'w')


for topic, msg, t in bag.read_messages(topics=['/joint_states']):
    same = True
    if topic == "/joint_states":
        pos_str = ""
        for pos in msg.position:
            pos_str = pos_str + str(pos)+","
        pos_str = pos_str[:-1]
        pos_str = pos_str + "\n"
        target.write(pos_str)

target.close()
bag.close()