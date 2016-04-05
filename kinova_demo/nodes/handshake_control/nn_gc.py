#!/usr/bin/python
import rospy 
from keras.models import model_from_json
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState
import numpy as np 

class nn_gc(object):

	def force_callback(self, data):
		
		new_wrench = WrenchStamped()
		new_wrench.header.stamp = data.header.stamp
		new_wrench.wrench.force.x = data.wrench.force.x - self.corrected_ft_data[0,0]
		new_wrench.wrench.force.y = data.wrench.force.y - self.corrected_ft_data[0,1]
		new_wrench.wrench.force.z = data.wrench.force.z - self.corrected_ft_data[0,2]

		self.wrench_publisher.publish(new_wrench)

	def joint_callback(self,data):
		self.corrected_ft_data = self.nn_model.predict(np.array([data.position[0:6]]))

	def __init__(self):
		self.corrected_ft_data = [0, 0, 0]
		self.nn_model = model_from_json(open("gravity_comp_model.json").read())
		self.nn_model.load_weights("gc_weights.h5")
		self.wrench_publisher = rospy.Publisher('/nn_gc', WrenchStamped, queue_size=1)
		rospy.Subscriber('/hand_ft_sensor', WrenchStamped, callback=self.force_callback)
		rospy.Subscriber('/joint_states', JointState, callback=self.joint_callback )


if __name__ == '__main__':
    rospy.init_node('nn_gc')
    test = nn_gc()
    rospy.spin()
        
