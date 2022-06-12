#!/usr/bin/python3
import rospy 
from ex5.srv import Dodaj 


def main(): 
	rospy.init_node('client') 
	rospy.wait_for_service('add') 
	try: 
		add_proxy = rospy.ServiceProxy('add', Dodaj)
		res = add_proxy(1.0, 2.0)
		rospy.loginfo("%f" % res.suma) 
		
	except rospy.ServiceException as e: 
		print("Service error: %s" % e)


if __name__ == "__main__": 
	try: 
		main() 
	except rospy.ROSInterruptException: 
		pass
