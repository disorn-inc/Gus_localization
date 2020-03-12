#! /usr/bin/env python

import rospy
#from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose

rospy.init_node('node_intermediate',anonymous = True)

def callback(msg):

	pub.publish(msg)

#sub = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, callback)
#pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

#sub = rospy.Subscriber('/imu/data/', Imu, callback)
#pub = rospy.Publisher('/turtle1/sensors/imu', Imu, queue_size=10)

sub = rospy.Subscriber('/turtle1/pose/', Pose, callback)
pub = rospy.Publisher('/turtle1/poseNEDtoENU', Pose, queue_size=10)

rospy.spin()
