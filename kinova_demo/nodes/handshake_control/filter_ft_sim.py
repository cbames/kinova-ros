#! /usr/bin/env python
# JR3 server for use in the simulated R2
# This node publishes the compensated force-torque for the forearm sensors.
# mlanighan@traclabs.com
# 2015
import roslib; roslib.load_manifest('kinova_demo')
import rospy
import tf
from geometry_msgs.msg import WrenchStamped, Vector3Stamped, Wrench, Vector3
import numpy as np
from scipy import signal
from copy import copy, deepcopy

class SimJR3Server:

    def __init__(self):
        self.pub_raw = rospy.Publisher('/filtered_ft_data', WrenchStamped, queue_size=1)
        self.filter_length = 30
        self.filter_history = np.array([[0 for _ in range(self.filter_length)] for _ in range(6)] )
        self.filter_index = 0
        # on the real robot, the ft are reported in these frames, in sim they are slightly altered
        self.names = ['r2/left_joint4_output', 'r2/left_shoulder_roll',
                      'r2/right_joint4_output', 'r2/right_shoulder_roll']

        self.frame_id = 'JR3s'

        self.idx = 0
        self.last = rospy.Time.now()
        self.wrench_raw = Wrench()

        self.listener = tf.TransformListener()
        rospy.Subscriber('/ft_sensor_jaco2_joint_6', WrenchStamped, self.wrench_cb)
        
        self.rate = rospy.Rate(100)
        print "Starting Sim-JR3 server"

    def filter_wrench(self):
        filtered_wrench = Wrench()
        # update the filter history,
        # -1 multiplication due to bug in f/t plugin patched upstream in indigo branch, but not hydro
        # https://github.com/ros-simulation/gazebo_ros_pkgs/commit/b3fc5f39505ad84aaba762b7bc6fac4701927f32
        self.filter_history[0][self.filter_index] = self.wrench_raw.force.x
        self.filter_history[1][self.filter_index] = self.wrench_raw.force.y
        self.filter_history[2][self.filter_index] = self.wrench_raw.force.z
        self.filter_history[3][self.filter_index] = self.wrench_raw.torque.x
        self.filter_history[4][self.filter_index] = self.wrench_raw.torque.y
        self.filter_history[5][self.filter_index] = self.wrench_raw.torque.z

        # apply a low-pass filter to the data
        ret_array = [0, 0, 0, 0, 0, 0]
        for i in range(6):
            ret_array[i] = np.median(self.filter_history[i])

        # update return wrench
        filtered_wrench.force.x = ret_array[0]
        filtered_wrench.force.y = ret_array[1]
        filtered_wrench.force.z = ret_array[2]
        filtered_wrench.torque.x = ret_array[3]
        filtered_wrench.torque.y = ret_array[4]
        filtered_wrench.torque.z = ret_array[5]
        return filtered_wrench

    def wrench_cb(self, data):
        self.wrench_raw = deepcopy(data.wrench)
        self.last = data.header.stamp


    def bias_ft(self, wrench, wrench_frame, bias):
        vs = Vector3Stamped()
        vs.vector = copy(bias)
        vs.header.frame_id = '/world'
        t = self.listener.getLatestCommonTime(wrench_frame, vs.header.frame_id)
        vs.header.stamp = t

        self.listener.waitForTransform(wrench_frame, vs.header.frame_id, vs.header.stamp, rospy.Duration(1))
        bias = deepcopy(self.listener.transformVector3(wrench_frame, vs))
        nw = Wrench()
        nw.force.x = wrench.force.x - bias.vector.x
        nw.force.y = wrench.force.x - bias.vector.y
        nw.force.z = wrench.force.x - bias.vector.z
        return nw

    def run(self):
        w = WrenchStamped()
        w.wrench = self.filter_wrench()
        self.filter_index += 1
        if self.filter_index % self.filter_length == 0:
            self.filter_index = 0
        w.header.stamp = self.last
        w.header.frame_id = "jaco2_link_hand"
        self.pub_raw.publish(w)
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('sim_jr3_server')
    sim_jr3_server = SimJR3Server()
    while not rospy.is_shutdown():
        sim_jr3_server.run()