#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('topic',String,queue_size=10)
	rospy.init_node('Teleop_talker',anonymous = True)
	print "Nodo creado con exito"
	rate = rospy.Rate(10) #Frecuencia del nodo
	while not rospy.is_shutdown():
		hello = "Hola mundo %s" % rospy.get_time()
		pub.publish(hello)
		print hello
		rate.sleep()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
