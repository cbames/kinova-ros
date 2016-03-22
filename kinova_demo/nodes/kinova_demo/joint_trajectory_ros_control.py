#! /usr/bin/env python
"""ROS Control of Gazebo Jaco ARM."""

import roslib; roslib.load_manifest('kinova_demo')
import rospy

import sys

import math

import actionlib
import control_msgs.msg 
import trajectory_msgs.msg 

import goal_generators


def joint_angle_client(angle_set):
    """Send a joint angle goal to the action server."""
    action_address = '/jaco_controller/follow_joint_trajectory'
    client = actionlib.SimpleActionClient(action_address,
                                          control_msgs.msg.FollowJointTrajectoryAction)
    client.wait_for_server()

    goal = control_msgs.msg.FollowJointTrajectoryActionGoal()

    goal.goal_id = 'test'
    goal.goal.trajectory.joint_names = ['jaco2_joint_1', 'jaco2_joint_2', 'jaco2_joint_3', 'jaco2_joint_4', 'jaco2_joint_5', 'jaco2_joint_6']

   # -1.6391335729986558, -1.7843264524686953, 0.8922353978998888, -1.1923773811773648, 1.6921760187458763, 3.3914919723742556

    goal.goal.trajectory.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
    goal.goal.trajectory.points[0].positions = angle_set# [-1.6391335729986558, -1.7843264524686953, 0.8922353978998888, -1.1923773811773648, 1.6921760187458763, 3.1914919723742556]
    goal.goal.trajectory.points[0].time_from_start.secs = 1.0
    #goal.goal.trajectory.points[0].velocities = [ 0, 0, 0, 0, 0, 0]
    #goal.goal.trajectory.points[0].accelerations = [ 0, 0, 0, 0, 0, 0]

    print('goal: {}'.format(goal.goal))

    client.send_goal(goal.goal)
    if client.wait_for_result(rospy.Duration(20.0)):
        return client.get_result()
    else:
        print('        the joint angle action timed-out')
        client.cancel_all_goals()
        return None


if __name__ == '__main__':
    if len(sys.argv) not in [3, 4, 8] or 'help' in str(sys.argv):
        print('Usage:')
        print('    joint_angle_workout.py node_name random num          - randomly generate num joint angle sets')
        print('    joint_angle_workout.py node_name file_path           - use poses from file')
        print('    joint_angle_workout.py node_name j1 j2 j3 j4 j5 j6   - use these specific angle')
        exit()

    try:
        rospy.init_node(str(sys.argv[1]) + '_joint_angle_workout')

        if str(sys.argv[2]) == 'random' and len(sys.argv) == 4:
            print('Using {} randomly generated joint angle sets'.format(int(sys.argv[3])))
            angles = goal_generators.random_joint_angles_generator(int(sys.argv[3]))
        elif len(sys.argv) == 3:
            print('Using joint angles from file: {}'.format(sys.argv[2]))
            angles = goal_generators.joint_angles_from_file(str(sys.argv[2]))
        else:
            print('Using the specified joint angles:')
            raw_angles = [float(n) for n in sys.argv[2:]]
            angles = [raw_angles]

        for angle_set in angles:
            print('    angles: {}'.format(angle_set))
            result = joint_angle_client(angle_set)

        print('Done!')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
