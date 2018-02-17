#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import tf2_ros
 
if __name__ == '__main__':
	rospy.init_node('tf2_simon_listener')
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)

	cmd_vel = rospy.Publisher('simon/cmd_vel', Twist, queue_size = 1)

	rate = rospy.Rate(10.0)

	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform('base_link', 'simon', rospy.Time(0))
			#print(trans)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rate.sleep()
			continue

		msg = Twist()

		msg.angular.z = math.atan2(trans.transform.translation.y, trans.transform.translation.x)
		msg.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)

		print(msg)
		#cmd_vel.publish(msg)
		ifmsg.angular.z <= 0.05 or msg.angular.z >= -0.05:
			cmd_vel.publish(msg)
		else:
			msg.angular.x = 0
			cmd.vel.publish(msg)
	
		
		rate.sleep()




