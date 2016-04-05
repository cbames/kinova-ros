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

from force_control import ForceControl
from kinova_demo.msg import ForceAction, ForceGoal

class ForceServer:

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
        self.ac_name = 'force_controller'

        self.action_server = actionlib.SimpleActionServer(self.ac_name, ForceAction, auto_start=False)
        self.action_server.register_goal_callback(cb=self.goal_cb)
        self.action_server.register_preempt_callback(cb=self.preempt_cb)


        self.joint_action_address = '/jaco_controller/follow_joint_trajectory'
        self.joint_client = actionlib.SimpleActionClient(self.joint_action_address,
                                              control_msgs.msg.FollowJointTrajectoryAction)
        self.joint_client.wait_for_server()
        
        self.wrench_at_tip = np.mat(np.zeros(6)).T
        
        # Call action for joint position control instead 
        #self.pub_joint_deltas = rospy.Publisher('/joint_refs', LabeledJointTrajectory, queue_size=1)
        
        # Need to listen to DMP for force 
        self.pub_goal = rospy.Publisher('/goal_at_control', WrenchStamped, queue_size=1)
        
        
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
        self.jnt_st_name_position = {}
        for name, position in names_positions:
            self.jnt_st_name_position[name] = position

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
        
        #print("arm:",arm)
        #This is visualization stuff which we might need for debugging purposes
        # tip = WrenchStamped()
        # tip.header = deepcopy(data.header)
        # tip.header.frame_id = self.base_frame # This seems wrong, but will only affect the visualization 
        # tip.wrench.force = Vector3((arm[0, 0]), (arm[1, 0]), (arm[2, 0]))
        # tip.wrench.torque = Vector3((arm_t[0, 0]), (arm_t[1, 0]), (arm_t[2, 0]))
        # self.pub_wrench_at_tip.publish(tip)
        #self.wrench_at_tip = np.mat([[tip.wrench.force.x], [tip.wrench.force.y], [tip.wrench.force.z], [0], [0], [0]])
        self.wrench_at_tip = np.mat( [ [arm[0, 0]], [arm[1, 0]], [arm[2,0]], [0], [0], [0]])

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

    def populate_jnt_command(self, deltas):
        # make a new joint command

        goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

        goal.goal_id = 'test'

        goal.goal.trajectory.points.append(JointTrajectoryPoint())
        #goal.goal.trajectory.joint_names = []
        #goal.goal.trajectory.points[0].positions = []

        #print("deltas:",deltas)

        for i in range(len(self.names)):
         if self.names[i] in self.jnt_st_name_position.keys():
            goal.goal.trajectory.joint_names.append(self.names[i])
            goal.goal.trajectory.points[0].positions.append(self.jnt_st_name_position[self.names[i]] + deltas[i] ) # This is the line for the deltas to be added in
         else: 
             return  False
        goal.goal.trajectory.points[0].time_from_start.secs = 1.0
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
            print 'Current Error Mag.: ' + str(np.sqrt(err))
            #if np.sqrt(err) < 1:
            #    self.state = ForceControl.CONVERGED
            #    return
            # compute joint deltas
            deltas = self.control.compute_jnt_deltas()
            # populate joint command and publish
            state = self.populate_jnt_command(deltas)
            if not state:
                return

        elif self.state == ForceControl.CONVERGED:
            print "CONVERGED!"
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