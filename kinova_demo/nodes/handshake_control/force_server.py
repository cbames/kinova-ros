#! /usr/bin/env python
# Class to create an actionlib force controller for an arm of NASA-GM Robonaut 2
# mlanighan@traclabs.com
# 2015
import rospy
import numpy as np
import tf
from tf.transformations import quaternion_matrix
from tf import TransformListener
from urdf_parser_py.urdf import URDF
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import WrenchStamped, Vector3
import actionlib
from copy import deepcopy
import socket
import actionlib
import control_msgs.msg 
import math 
from dmp.srv import *
from dmp.msg import *

from force_control import ForceControl
from kinova_demo.msg import ForceAction, ForceGoal

class ForceServer:

    #Learn a DMP from demonstration data
    def makeLFDRequest(self,dims, traj, dt, K_gain, 
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
    def makeSetActiveRequest(self,dmp_list):
        try:
            sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
            sad(dmp_list)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    #Generate a plan from a DMP
    def makePlanRequest(self,x_0, x_dot_0, t_0, goal, goal_thresh, 
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

    def __init__(self, arm):
        """
        Creates an actionlib server for force control on an arm.
        :param arm: need to make this more generic... 
        """
        robot = URDF.from_parameter_server()

        self.base_frame = 'jaco2_api_origin'
        self.tip_frame = 'jaco2_end_effector'
        self.control = ForceControl(self.base_frame, self.tip_frame, robot)

        self.name_position_dict = {}
        self.jnt_st_name_position = {}
        self.jnt_st_name_velocity = {}
        self.ac_name = 'force_controller'

        self.action_server = actionlib.SimpleActionServer(self.ac_name, ForceAction, auto_start=False)
        self.action_server.register_goal_callback(cb=self.goal_cb)
        self.action_server.register_preempt_callback(cb=self.preempt_cb)


        self.joint_action_address = '/jaco_controller/follow_joint_trajectory'
        self.joint_client = actionlib.SimpleActionClient(self.joint_action_address,
                                              control_msgs.msg.FollowJointTrajectoryAction)
        self.joint_client.wait_for_server()
        
        self.wrench_at_tip = np.mat(np.zeros(6)).T

        self.force_index = 0 
        self.force_traj = np.genfromtxt("/home/ubuntu_ros/catkin_ws/src/jaco-ros/kinova_demo/nodes/handshake_control/ben_traj.txt",delimiter=",").tolist()

        #Now, generate a plan
        # x_0 = [-1.49572028597,-0.505756026042,-0.12243649715,-1.07337658333,1.51367516971,-3.06146367312,0.0,0.0,0.0]          #Plan starting at a different point than demo 
        # x_dot_0 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]   
        # t_0 = 0                
        # goal = [-1.5496200909,-0.10433450521,-0.451710138562,-1.10312646443,1.73977442581,-3.06146367312,0.0,0.0,0.0]         #Plan to a different goal than demo
        # goal_thresh = [0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
        # seg_length = -1          #Plan until convergence to goal
        # tau = 8       #Desired plan should take twice as long as demo
        # dt = 0.025
        # integrate_iter = 1       #dt is rather large, so this is > 1  
        # self.force_traj = self.makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh, 
        #                        seg_length, tau, dt, integrate_iter)



        self.last_time = rospy.Time.now()

        
        # Call action for joint position control instead 
        #self.pub_joint_deltas = rospy.Publisher('/joint_refs', LabeledJointTrajectory, queue_size=1)
        
        # Need to listen to DMP for force 
        self.pub_goal = rospy.Publisher('/goal_at_control', WrenchStamped, queue_size=1, tcp_nodelay=True)
        
        
        self.listener = TransformListener(True, rospy.Duration(2))
        
        self.state = ForceControl.NO_REFERENCE
        self.goal = ForceGoal()
        self.rate = rospy.Rate(100) # NOTE: may want to make this faster

        # build out name list
        self.names = list()
        base_string = 'jaco2_joint_'
        for i in range(6):
            self.names.append(base_string+str(i+1))

        print("names:", self.names)

        # set up subscribers
        rospy.Subscriber('/joint_states', JointState, callback=self.joint_state_cb) 
        #rospy.Subscriber('joint_command_refs', JointCommand, callback=self.joint_cmd_cb) # should probably get this from the DMP 
        
        # rostopic for end effector force 
        rospy.Subscriber('/hand_ft_sensor', WrenchStamped, self.wrench_cb)
        # start action_server
        self.action_server.start()
        print 'Started arm force controller!'

        from dmp.srv import *
        from dmp.msg import *




    def preempt_cb(self):
        self.state = ForceControl.NO_REFERENCE
        self.goal = None
        rospy.loginfo('Received preempt request!')
        rospy.loginfo('Stopping this shit now!  YOLO!')


    def joint_cmd_cb(self, data):
        # copy current joint cmd so we don't wipe anything out inadvertently
        self.local_jnt_cmd = deepcopy(data)

        # Create dictionary of joint name to desired joint position
        names_positions = zip(self.local_jnt_cmd.name, self.local_jnt_cmd.desiredPosition) 
        for name, position in names_positions:
            self.name_position_dict[name] = position

    def joint_state_cb(self, data):
        
        # Create dictionary of joint name to desired joint position
        names_positions = zip(data.name, data.position) 
        names_velocities = zip(data.name, data.velocity)

        self.jnt_st_name_position = {}
        #self.jnt_st_name_velocity = {}
        
        for name, position in names_positions:
            self.jnt_st_name_position[name] = position

        for name, velocity in names_velocities:
            self.jnt_st_name_velocity[name] = velocity

        #print "velocity:", self.jnt_st_name_velocity

        # update only the joints we care about
        current_pos = list()
        for i in range(len(self.names)):
            if self.names[i] in self.jnt_st_name_position.keys():
                current_pos.append(self.jnt_st_name_position[self.names[i]])
            else: 
                rospy.logwarn('Joint Not found in joint_states message')
        self.control.update_jnts(current_pos)

    def wrench_cb(self, data):
        # Transform Force/Torque to end effector force/torque
        arm = self.transform_force(data.wrench, data.header.frame_id)
        arm_t = self.transform_torque(data.wrench, data.header.frame_id)
        
        threshold = 6
        t_thresh  = 4 
        #print("arm:",arm)
        #This is visualization stuff which we might need for debugging purposes
        # tip = WrenchStamped()
        # tip.header = deepcopy(data.header)
        # tip.header.frame_id = self.base_frame # This seems wrong, but will only affect the visualization 
        # tip.wrench.force = Vector3((arm[0, 0]), (arm[1, 0]), (arm[2, 0]))
        # tip.wrench.torque = Vector3((arm_t[0, 0]), (arm_t[1, 0]), (arm_t[2, 0]))
        # self.pub_wrench_at_tip.publish(tip)
        #self.wrench_at_tip = np.mat([[tip.wrench.force.x], [tip.wrench.force.y], [tip.wrench.force.z], [0], [0], [0]])
        for i in range(len(arm[:,0])): 
            f = arm[i,0]
            f_new = f - np.sign(f)*threshold 
            if np.sign(f_new) != np.sign(f):
                f = 0 
            else: 
                f = f_new
            arm[i,0] = f

        for i in range(len(arm_t[:,0])): 
            t = arm_t[i,0]
            t_new = t - np.sign(t)*t_thresh 
            if np.sign(t_new) != np.sign(t):
                t = 0 
            else: 
                t = t_new
            arm_t[i,0] = t



        self.wrench_at_tip = np.mat( [ [arm[0, 0]], [arm[1, 0]], [arm[2,0]], [arm_t[0, 0]], [arm_t[1, 0]], [arm_t[2,0]]])

        #print "wrench at tip:", self.wrench_at_tip

    def transform_force(self, wrench, frame):
        # need to transform force data to the correct frame (tip of chain)
        tmp = np.mat([[0], [0], [0], [1]])

        try:
            #print("base_frame:", self.base_frame)
            #print("frame:", frame)
            position, quaternion = self.listener.lookupTransform(self.base_frame, frame, rospy.Time(0))
            cf_rot_data = quaternion_matrix(quaternion)
            tmp = np.mat([[wrench.force.x], [wrench.force.y], [wrench.force.z], [1]])
            tmp = cf_rot_data*tmp
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception")
            pass
        return tmp

    def transform_torque(self, wrench, frame):
        # This transforms the torque to the correct frame
        tmp = np.mat([[0], [0], [0], [1]])

        try:
            position, quaternion = self.listener.lookupTransform(self.base_frame, frame, rospy.Time(0))
            cf_rot_data = quaternion_matrix(quaternion)
            t = np.mat([[wrench.torque.x], [wrench.torque.y], [wrench.torque.z], [1]])
            f = np.mat([[wrench.force.x], [wrench.force.y], [wrench.force.z], [1]])
            tmp = np.mat([[0], [0], [0], [1]])
            x = np.zeros((4, 4))
            x[3, 3] = 1
            x[0, 1] = -position[2]
            x[0, 2] = position[1]
            x[1, 0] = position[2]
            x[1, 2] = -position[0]
            x[2, 0] = -position[1]
            x[2, 1] = position[0]
            tmp = cf_rot_data*t + x*(cf_rot_data*f)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return tmp

    def goal_cb(self):

        self.action_server.accept_new_goal()
        # idk why this is so buried :-/ seems weird
        self.goal = self.action_server.current_goal.goal.goal
        print 'Received a goal!'
        self.state = ForceControl.TRANSIENT
        print 'Accepted goal, now sending commands!'

        self.last_time = rospy.Time.now()


    def simplify_angle(self,angle):

        previous_rev = math.floor(angle/(2.0*math.pi)) * 2.0*math.pi;
        next_rev = math.ceil(angle/(2.0*math.pi)) * 2.0*math.pi;
        
        if (math.fabs(angle - previous_rev) < math.fabs(angle - next_rev)):
            return angle - previous_rev;

        return angle - next_rev;
        
    def nearest_equivalent(self, desired, current):
    
      #calculate current number of revolutions
      previous_rev = math.floor(current / (2.0*math.pi))
      next_rev = math.ceil(current / (2.0*math.pi))
      current_rev = None
      
      if (math.fabs(current - previous_rev*2.0*math.pi) < math.fabs(current - next_rev*2.0*math.pi)):
        current_rev = previous_rev
      else:
        current_rev = next_rev
      
      #determine closest angle
      lowVal = (current_rev - 1.0)*2.0*math.pi + desired
      medVal = current_rev*2.0*math.pi + desired
      highVal = (current_rev + 1.0)*2.0*math.pi + desired
      
      if (math.fabs(current - lowVal) <= math.fabs(current - medVal) and math.fabs(current - lowVal) <= math.fabs(current - highVal)):
        return lowVal
      
      if (math.fabs(current - medVal) <= math.fabs(current - lowVal) and math.fabs(current - medVal) <= math.fabs(current - highVal)):
        return medVal
      
      return highVal

    def populate_jnt_command(self, deltas):
        # make a new joint command

        goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

        goal.goal_id = 'test'
        goal.header.stamp = rospy.Time.now()+ rospy.Duration(0.02)
        goal.goal.trajectory.points.append(JointTrajectoryPoint())
        #goal.goal.trajectory.joint_names = []
        #goal.goal.trajectory.points[0].positions = []

        mode = 0 # 0 = position, 1 = force, 2 = both 

        dt = 0.001
        kp = 2
        num_beginning = 60 
        if self.force_index < num_beginning: 
            kp = 2

        if mode == 2: 
            kp = 0.2


        pos_traj_deltas = np.zeros(6)

        if self.force_index < len(self.force_traj):#len(self.force_traj.plan.points):#
            for i in range(len(self.names)):
                if self.names[i] in self.jnt_st_name_position.keys():
                    
                    current_pos = self.jnt_st_name_position[self.names[i]]
                    setpoint = self.nearest_equivalent(self.simplify_angle(self.force_traj[self.force_index][i]), current_pos) 
                    #setpoint = self.nearest_equivalent(self.simplify_angle(self.force_traj.plan.points[self.force_index].positions[i]), current_pos) 


                    if i == 0:
                        pos_traj_deltas[i] = 0 
                    elif i == 2:
                        pos_traj_deltas[i] =- kp*(setpoint - current_pos)
                    elif i == 1 and self.force_index > num_beginning :
                        pos_traj_deltas[i] = kp*(setpoint +.15 - current_pos)
                    elif i == 4 or i == 5 or i == 3:
                        pos_traj_deltas[i] = 0 
                    else:
                        pos_traj_deltas[i] = kp*(setpoint - current_pos)


            cur_time = rospy.Time.now()

            if cur_time - self.last_time >=rospy.Duration(dt):
                self.last_time = cur_time
                self.force_index = self.force_index + 1 

            

        #print "velocity:",self.jnt_st_name_velocity
        for i in range(len(self.names)):
         if self.names[i] in self.jnt_st_name_position.keys():
            goal.goal.trajectory.joint_names.append(self.names[i])
            
            #goal.goal.trajectory.points[0].positions.append(self.jnt_st_name_position[self.names[i]] + deltas[i] ) # This is the line for the deltas to be added in
            
            if mode == 0 or self.force_index < num_beginning:
                goal.goal.trajectory.points[0].positions.append( pos_traj_deltas[i] ) 
            elif mode == 1: 
                goal.goal.trajectory.points[0].positions.append( self.jnt_st_name_velocity[self.names[i]] +  deltas[i]  )
            else: 
                goal.goal.trajectory.points[0].positions.append( self.jnt_st_name_velocity[self.names[i]] +  deltas[i] + pos_traj_deltas[i])
            
         else: 
             return  False
        goal.goal.trajectory.points[0].time_from_start = rospy.Duration(0.01)
        #print('goal: {}'.format(goal.goal))
        self.joint_client.send_goal(goal.goal)
 
        return  True

    def run(self):
        if self.state == ForceControl.NO_REFERENCE:
            # wait for a goal
            return

        elif self.state == ForceControl.TRANSIENT:
            # update the goal in the control frame
            last = self.listener.getLatestCommonTime(self.base_frame, self.goal.wrench.header.frame_id)
            goal_control_f = self.transform_force(self.goal.wrench.wrench, self.goal.wrench.header.frame_id)
            goal_control_t = self.transform_torque(self.goal.wrench.wrench, self.goal.wrench.header.frame_id)

            goal_vis = WrenchStamped()
            goal_vis.header.frame_id = self.base_frame
            goal_vis.header.stamp = last
            goal_vis.wrench.force = Vector3(goal_control_f[0, 0], goal_control_f[1, 0], goal_control_f[2, 0])
            goal_vis.wrench.torque = Vector3(goal_control_t[0, 0], goal_control_t[1, 0], goal_control_t[2, 0])
            self.pub_goal.publish(goal_vis)
            npgoal = np.mat([[goal_control_f[0, 0]], [goal_control_f[1, 0]], [goal_control_f[2, 0]], [0], [0], [0]])
            self.control.update_goal(npgoal)

            # check for convergence
            err = self.control.update_error(self.wrench_at_tip)
            # print 'Current Error Mag.: ' + str(np.sqrt(err))
            # print 'current goal:' + str(self.goal.wrench.wrench)
            # if np.sqrt(err) < 5:
            #    self.goal.wrench.wrench.force.x = 2.0*self.force_traj[self.force_index][0]
            #    self.goal.wrench.wrench.force.y = 2.0*self.force_traj[self.force_index][1]
            #    self.goal.wrench.wrench.force.z = 2.0*self.force_traj[self.force_index][2]
            #    self.goal.wrench.wrench.torque.x = 0#3.0*self.force_traj[self.force_index][3]
            #    self.goal.wrench.wrench.torque.y = 0#3.0*self.force_traj[self.force_index][4]
            #    self.goal.wrench.wrench.torque.z = 0#3.0*self.force_traj[self.force_index][5]
            #    self.force_index = self.force_index + 1 


            #    self.state = ForceControl.CONVERGED
            #    return
            # compute joint deltas
            deltas = self.control.compute_jnt_deltas()
            # populate joint command and publish
            state = self.populate_jnt_command(deltas)
            if not state:
                return

        elif self.state == ForceControl.CONVERGED:
            #print "CONVERGED!"
            self.action_server.set_succeeded()
            self.state = ForceControl.NO_REFERENCE
            # err = self.control.update_error(self.wrench_at_tip)
            # # success!
            # print 'Current Error Mag.: ' + str(np.sqrt(err))
            # if np.sqrt(err) > 2:
            #     self.state = ForceControl.TRANSIENT
            return

if __name__ == '__main__':
    rospy.init_node('force_server')
    fc = ForceServer('right')
    while not rospy.is_shutdown():
        fc.run()
        fc.rate.sleep()