#!/usr/bin/env python
import roslib; 
roslib.load_manifest('dmp')
import rospy 
import numpy as np
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


if __name__ == '__main__':
    rospy.init_node('dmp_tutorial_node')

    #Create a DMP from a 2-D trajectory
    dims = 9#2                
    dt = 0.025                
    K = 250                 
    D = 2.0 * np.sqrt(K)      
    num_bases = 10000          
    
    #np.genfromtxt("/home/ubuntu_ros/catkin_ws/src/jaco-ros/kinova_demo/nodes/handshake_control/ben_traj.txt",delimiter=",").tolist()
    #print [[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    traj = np.genfromtxt("/home/ubuntu_ros/catkin_ws/src/jaco-ros/kinova_demo/nodes/handshake_control/ben_traj.txt",delimiter=",").tolist()#[[1.0,1.0],[2.0,2.0],[3.0,4.0],[6.0,8.0]]
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    #Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    #Now, generate a plan
    x_0 = [-1.65260863548,-2.60054058547,-2.52559417239,3.08328274152,2.42878427202,1.75167583119,0.838176919978,0.839433557039,-0.0]          #Plan starting at a different point than demo 
    x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   
    t_0 = 0                
    goal = [-1.64779630339,-2.70771470985,-2.5843064881,-3.03687289847,2.41212406888,1.75167583119,0.838176919978,0.839433557039,-0.0]         #Plan to a different goal than demo
    goal_thresh = [0.2,0.2]
    seg_length = -1          #Plan until convergence to goal
    tau = resp.tau       #Desired plan should take twice as long as demo
    dt = 0.1
    integrate_iter = 1       #dt is rather large, so this is > 1  
    plan = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
                           seg_length, tau, dt, integrate_iter)


    #print plan
    print "done"