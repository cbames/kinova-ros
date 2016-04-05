import rosbag

from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

bag = rosbag.Bag('/home/ubuntu_ros/handshake_comp/handshake_gravity_comp.bag')

prev_joint_state = None 
sec_prev_joint_state = None 
third_prev_joint_state = None
fourth_prev_joint_state = None

prev_hand_ft = None 
jnt_state_tmp = None

target = open("gravity_comp.txt", 'w')


for topic, msg, t in bag.read_messages(topics=['/joint_states', '/hand_ft_sensor']):
    same = True
    if topic == "/joint_states":
        
        # Make sure that the joints have been stationary before grabbing point 
        if sec_prev_joint_state == None or prev_joint_state == None or third_prev_joint_state == None or fourth_prev_joint_state == None:
            same = False
        
        for i in range(len(msg.position)):
                same = same and ( msg.position[i] == prev_joint_state.position[i] == sec_prev_joint_state.position[i] == third_prev_joint_state.position[i] == fourth_prev_joint_state.position[i])

        if same:
            jnt_state_tmp = prev_joint_state

        prev_joint_state = msg 
        sec_prev_joint_state = prev_joint_state
        third_prev_joint_state = sec_prev_joint_state
        fourth_prev_joint_state = third_prev_joint_state

    elif topic == "/hand_ft_sensor" and same:

        if jnt_state_tmp != None and jnt_state_tmp.header.stamp == prev_hand_ft.header.stamp:



            #Joints first followed by forces x,y,z
            for jnt in jnt_state_tmp.position:
                target.write(str(jnt)+",")
        
            target.write("jt_time:"+str(jnt_state_tmp.header.stamp)+",")
            target.write("ft_time:"+str(prev_hand_ft.header.stamp)+",")

            target.write(str(prev_hand_ft.wrench.force.x)+",")
            target.write(str(prev_hand_ft.wrench.force.y)+",")
            target.write(str(prev_hand_ft.wrench.force.z)+"\n")


        prev_hand_ft = msg 


target.close()
bag.close()