#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty

def sequence(message):
	name = []
	msg.joint_name = name
	msg.points.positions = []

if __name__ == "__main__":
    # Entry point
    rospy.init_node('gorgon_node')

    msg = JointTrajectory()
 
    trajectory = rospy.Publisher("arms_command", JointTrajectory, qeue_size=1)
    rospy.Subscriber("trigger_arm_seq", Empty, sequence) 
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
rospy.spin()



