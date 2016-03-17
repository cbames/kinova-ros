#! /usr/bin/env python

import rospy
from kinova_demo.msg import ForceAction, ForceGoal
import actionlib

if __name__ == '__main__':
    rospy.init_node('force_client')

    ac = actionlib.SimpleActionClient('force_controller', ForceAction)
    ac.wait_for_server()
    goal = ForceGoal()
    goal.wrench.wrench.force.x = 6
    goal.wrench.wrench.force.y = 0
    goal.wrench.wrench.force.z = 0
    goal.wrench.wrench.torque.x = 0
    goal.wrench.wrench.torque.y = 0
    goal.wrench.wrench.torque.z = 0

    goal.wrench.header.stamp = rospy.Time.now()
    goal.wrench.header.frame_id = 'jaco2_api_origin'
    print 'Sending goal!'
    print goal
    ac.send_goal_and_wait(goal)
