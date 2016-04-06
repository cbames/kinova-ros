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


def joint_angle_client(angle_set, velocity_set):
    """Send a joint angle goal to the action server."""
    action_address = '/jaco_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(action_address,
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

    goal.goal_id = 'test'
    goal.goal.trajectory.joint_names = ['jaco2_joint_1', 'jaco2_joint_2', 'jaco2_joint_3', 'jaco2_joint_4', 'jaco2_joint_5', 'jaco2_joint_6']

    goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    goal.goal.trajectory.points[0].positions = angle_set[0:6]
    goal.goal.trajectory.points[0].time_from_start.secs = 0.
    goal.goal.trajectory.points[0].velocities = velocity_set[0:6]

    print('goal: {}'.format(goal.goal))

    client.send_goal_and_wait(goal.goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        print client.get_result()
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


if __name__ == '__main__':

    rospy.init_node('handshake_ros_control')

    #Now, generate a plan
    x_0 = [1.58427125612,0.360246865919,-1.85858310683,-1.74453669525,3.1285026842,3.14,-0.00251327412287,-0.00251327412287,-0.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   
    t_0 = 0                
    goal = [1.58523372254,0.426514835956,-2.06937017658,-1.74572979142,3.11422281444,3.14,-0.00251327412287,-0.00251327412287,-0.0]         #Plan to a different goal than demo
    goal_thresh = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    seg_length = -1          #Plan until convergence to goal
    tau = 1.0       #Desired plan should take twice as long as demo
    dt = 0.025
    integrate_iter = 5       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)

    action_address = '/jaco_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(action_address,
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

    #traj = np.genfromtxt("/home/ubuntu_ros/catkin_ws/src/jaco-ros/kinova_demo/nodes/handshake_control/ben_traj.txt",delimiter=",").tolist()


    goal.header.stamp = rospy.Time.now()+rospy.Duration(5)
    goal.goal_id = 'test'
    goal.goal.trajectory.header.stamp = rospy.Time.now()
    goal.goal.trajectory.joint_names = ['jaco2_joint_1', 'jaco2_joint_2', 'jaco2_joint_3', 'jaco2_joint_4', 'jaco2_joint_5', 'jaco2_joint_6']
    
    
    for i in range(len(plan.plan.points)):
    #for i in range(len(traj)):
        dt = (i+2)*0.3
        goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        goal.goal.trajectory.points[i].positions = list(plan.plan.points[i].positions[0:6])#traj[i][0:6]
        goal.goal.trajectory.points[i].positions[1] = goal.goal.trajectory.points[i].positions[1] - .15
        goal.goal.trajectory.points[i].time_from_start = rospy.Duration(dt)
        #goal.goal.trajectory.points[i].velocities = plan.plan.points[i].velocities[0:6]

    print('goal: {}'.format(goal.goal))

    client.send_goal(goal.goal)
    if client.wait_for_result(rospy.Duration(10)+goal.goal.trajectory.points[-1].time_from_start):
        print client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()



