#!/usr/bin/python
from urdf_parser_py.urdf import URDF
import PyKDL as kdl
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, Vector3
import rospy 
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
import tf
from tf.transformations import quaternion_matrix
from tf import TransformListener

class ee_cart(object):
    """Random class to hold stuff"""
    # convert kdl data to numpy to make it easier to use
    def jac_to_np(self, data):
        mat = np.mat(np.zeros((data.rows(), data.columns())))
        for i in range(data.rows()):
            for j in range(data.columns()):
                mat[i, j] = data[i, j]
        return mat

    def joint_state_cb(self,data):
        
        jnt_pos = kdl.JntArray(self.chain.getNrOfJoints())

        for i in range(6):
            jnt_pos[i] = data.position[i]

        print("jnt_pos",jnt_pos)

        jacobian =  kdl.Jacobian(self.chain.getNrOfJoints())
        self.jnt_to_jac.JntToJac(jnt_pos, jacobian)
        np_jacobian = self.jac_to_np(jacobian)

        joint_efforts =np.mat(data.effort[0:6]).T

        print("joint_efforts:", joint_efforts)
        print("np_jacobian:", np_jacobian)

        cart_force = np_jacobian*joint_efforts


        wrench = WrenchStamped()
        wrench.header.stamp = rospy.Time.now()

        wrench.wrench.force.x = cart_force[0]
        wrench.wrench.force.y = cart_force[1]
        wrench.wrench.force.z = cart_force[2]

        transformed_wrench = self.transform_force(wrench.wrench, "jaco2_link_hand")

        wrench.wrench.force.x = transformed_wrench[0]
        wrench.wrench.force.y = transformed_wrench[1]
        wrench.wrench.force.z = transformed_wrench[2]

        wrench.header.frame_id="jaco2_api_origin"


        self.ee_cart_pub.publish(wrench)

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

    def __init__(self):
        #super(ee_cart, self).__init__()

        robot = URDF.from_parameter_server()
        root = "jaco2_api_origin"
        tip = "jaco2_link_hand"
        
        self.base_frame = 'jaco2_api_origin'

        tree = kdl_tree_from_urdf_model(robot)
        self.chain = tree.getChain(root, tip)
        self.jnt_to_jac = kdl.ChainJntToJacSolver(self.chain)  
        self.listener = TransformListener(True, rospy.Duration(2))


        self.ee_cart_pub = rospy.Publisher('/ee_cart', WrenchStamped, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, callback=self.joint_state_cb) 




if __name__ == '__main__':
    rospy.init_node('cart_force')
    test = ee_cart()
    rospy.spin()
        
                
