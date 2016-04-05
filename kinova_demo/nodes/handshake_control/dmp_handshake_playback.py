#! /usr/bin/env python
"""ROS Control of Gazebo Jaco ARM."""

import roslib; 
roslib.load_manifest('kinova_demo')
import rospy 
import numpy as np

import sys

import math

import actionlib
import control_msgs.msg 
import trajectory_msgs.msg 
from dmp.srv import *
from dmp.msg import *

#Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain, 
                   D_gain, num_bases):
    demotraj = DMPTraj()
        
    for i in range(len(traj)):
        pt = DMPPoint();
        pt.positions = traj[i]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)
            
    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims
        

    print demotraj
    print "Starting LfD..."
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "LfD done"    
            
    return resp;


#Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


#Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                    seg_length, tau, dt, integrate_iter):
    print "Starting DMP planning..."
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh, 
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    print "DMP planning done"   
            
    return resp;


# def joint_angle_client(angle_set, velocity_set):
#     """Send a joint angle goal to the action server."""
#     action_address = '/jaco_controller/follow_joint_trajectory'
#     client = actionlib.SimpleActionClient(action_address,
#                                           control_msgs.msg.FollowJointTrajectoryAction)
#     client.wait_for_server()

#     goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

#     goal.goal_id = 'test'
#     goal.goal.trajectory.joint_names = ['jaco2_joint_1', 'jaco2_joint_2', 'jaco2_joint_3', 'jaco2_joint_4', 'jaco2_joint_5', 'jaco2_joint_6']

#     goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
#     goal.goal.trajectory.points[0].positions = angle_set[0:6]
#     goal.goal.trajectory.points[0].time_from_start.secs = 0.
#     goal.goal.trajectory.points[0].velocities = velocity_set[0:6]

#     print('goal: {}'.format(goal.goal))

#     client.send_goal_and_wait(goal.goal)
#     if client.wait_for_result(rospy.Duration(20.0)):
#         print client.get_result()
#         return client.get_result()
#     else:
#         print('        the joint angle action timed-out')
#         client.cancel_all_goals()
#         return None


if __name__ == '__main__':

    rospy.init_node('handshake_ros_control')

    #Now, generate a plan
    # x_0 = [-1.65260863548,-2.60054058547,-2.52559417239,3.08328274152,2.42878427202,1.75167583119,0.838176919978,0.839433557039,-0.0]          #Plan starting at a different point than demo 
    # x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   
    # t_0 = 0                
    # goal = [-1.64779630339,-2.70771470985,-2.5843064881,-3.03687289847,2.41212406888,1.75167583119,0.838176919978,0.839433557039,-0.0]         #Plan to a different goal than demo
    # goal_thresh = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    # seg_length = -1          #Plan until convergence to goal
    # tau = 5.0       #Desired plan should take twice as long as demo
    # dt = 2.0
    # integrate_iter = 5       #dt is rather large, so this is > 1  
    # plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
    #                        seg_length, tau, dt, integrate_iter)

    action_address = '/jaco_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(action_address,
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

    traj = np.genfromtxt("/home/ubuntu_ros/catkin_ws/src/jaco-ros/kinova_demo/nodes/handshake_control/ben_traj.txt",delimiter=",").tolist()


    goal.header.stamp = rospy.Time.now()+rospy.Duration(5)
    goal.goal_id = 'test'
    goal.goal.trajectory.header.stamp = rospy.Time.now()
    goal.goal.trajectory.joint_names = ['jaco2_joint_1', 'jaco2_joint_2', 'jaco2_joint_3', 'jaco2_joint_4', 'jaco2_joint_5', 'jaco2_joint_6']
    #goal.goal.goal_tolerance.append( 0.1)
    
    # goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    # goal.goal.trajectory.points[0].positions= [-1.6343198604566784, -1.8088686171875, 0.9644218948993682, -1.1899963089633518, 1.7135945091387432, 3.14]
    # goal.goal.trajectory.points[0].time_from_start = rospy.Duration(1.0)


    # goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    # goal.goal.trajectory.points[1].positions= [-1.6343198604566784, -1.8088686171875, 0.9644218948993682, -1.1899963089633518, 1.7135945091387432, -3.14]
    # goal.goal.trajectory.points[1].time_from_start = rospy.Duration(2.0)


    #for i in range(len(plan.plan.points)):
    for i in range(len(traj)):
        dt = (i+1)*0.2
        goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        goal.goal.trajectory.points[i].positions = traj[i][0:6]#plan.plan.points[i].positions[0:6]
        goal.goal.trajectory.points[i].time_from_start = rospy.Duration(dt)
        #goal.goal.trajectory.points[i].velocities = plan.plan.points[i].velocities[0:6]

    print('goal: {}'.format(goal.goal))

    client.send_goal(goal.goal)
    if client.wait_for_result(rospy.Duration(1.0)+goal.goal.trajectory.points[-1].time_from_start):
        print client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()



