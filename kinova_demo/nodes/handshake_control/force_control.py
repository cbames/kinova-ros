#! /usr/bin/env python
# Class to provide Jacobian transpose force control.
# This just provides the basic computations, a node class is still needed to
# instantiate an instance and populate the required data via callbacks
# mlanighan@traclabs.com
# 2015

import PyKDL as kdl
import numpy as np
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from copy import deepcopy


class ForceControl:
    NO_REFERENCE, TRANSIENT, CONVERGED = range(-1, 2)

    def __init__(self, root, tip, robot):
        self.root = root
        self.tip = tip
        self.pos = kdl.Frame()

        self.robot = robot
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(root, tip)
        self.fk = kdl.ChainFkSolverPos_recursive(self.chain)
        

        self.jnt_pos = kdl.JntArray(self.chain.getNrOfJoints())
        self.j = kdl.Jacobian(self.chain.getNrOfJoints())

        self.goal = np.mat(np.zeros(6)).T
        self.error = np.mat(np.zeros(6)).T
        self.jnt_to_jac = kdl.ChainJntToJacSolver(self.chain)
        self.k = 0.018

    def update_jnts(self, data):
        i = 0
        #print("num of joints:", self.chain.getNrOfJoints())
        for num in data:

            self.jnt_pos[i] = num
            i += 1

    # returns the squared residual
    def update_error(self, wrench):
        # yay numpy!
        self.error = self.goal - wrench
        # print '---goal---'
        # print self.goal
        # print '---input---'
        # print wrench
        # print '---error---'
        # print self.error

        return np.dot(self.error.T, self.error)

    def set_goal(self, goal):
        #self.goal = deepcopy(goal)
        #print("setting goal:", goal)
        self.fk.JntToCart(self.jnt_pos, self.pos)

    def update_goal(self, goal):
        #print("updating goal:", goal)
        self.goal = deepcopy(goal)

    # convert kdl data to numpy to make it easier to use
    def jac_to_np(self, data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat

    '''
    This computation comes from the relationship:
                                delta_t = J^T * delta_f,
    where delta_t are changes in joint torques and delta_f are changes in a wrench.
    We know that an underlying controller exists at the joint level such that
                                delta_t = k * delta_q,
    where delta_q are changes in joint positions. As such, we can use the following:
                                delta_q = k^-1 J^T * f_error
    where f_error is the wrench residual (f_ref-f_obs).

    '''
    def compute_jnt_deltas(self):

        # need to stay in y-z plane (should generalize this)
        #cj = self.compute_control_jacobian()
        #print cj
        #print np.linalg.pinv(cj)
        #error = self.compute_potential(self.jnt_pos)
        #ns = np.mat(np.eye(self.chain.getNrOfJoints()))-np.linalg.pinv(cj)*cj
        self.jnt_to_jac.JntToJac(self.jnt_pos, self.j)
        npj = self.jac_to_np(self.j)

        #print "jacobian:", npj
        # yay numpy!
        #jac_pinv = None 

        lambda_dls = 0.5
        j_jT = np.dot(npj, npj.T)

        #print ("singularity check:"+"{0:0.10f}".format(abs(np.linalg.det(j_jT))))

        #jac_pinv = np.linalg.pinv(npj,rcond=0.05)
        jac_pinv = npj.T*np.linalg.inv(j_jT + lambda_dls*lambda_dls*np.identity(j_jT.shape[0]))


        deltas = self.k*jac_pinv*self.error

        lp = 0
        lock_wrist = True

        for delta in deltas:
            
            if lp == 0 or lp == 2:
                deltas[lp] = delta * -1.0
            if (lp == 3 or lp == 4 or lp == 5)  and lock_wrist:
                deltas[lp] = 0

            lp = lp+1


        #print("joint_deltas:",deltas)
        #print("error:", self.error)
        #return np.linalg.pinv(cj)*error# + ns*deltas

        return deltas

    def compute_potential(self, jnts):
        current_pos = kdl.Frame()
        self.fk.JntToCart(self.jnt_pos, current_pos)
        x_err = self.pos.p.x() - current_pos.p.x()
        y_err = self.pos.p.y() - current_pos.p.y()
        z_err = self.pos.p.z() - current_pos.p.z()
        return (x_err**2)+(y_err**2) + (z_err**2)

    def compute_control_jacobian(self):
        cntrl_jac = np.mat([[0], [0], [0], [0], [0], [0], [0]])
        local_jnt_pos = kdl.JntArray(self.chain.getNrOfJoints())
        for i in range(self.chain.getNrOfJoints()):
            local_jnt_pos[i] = self.jnt_pos[i]
        current_potential = self.compute_potential(self.jnt_pos)
        delta = 0.001
        for i in range(self.chain.getNrOfJoints()):
            # determine how offsetting this joint by delta would effect endpoint pos
            local_jnt_pos[i] += delta
            offset_pot = self.compute_potential(local_jnt_pos)
            cntrl_jac[i, 0] = current_potential-offset_pot
            local_jnt_pos[i] -= delta
        return cntrl_jac
