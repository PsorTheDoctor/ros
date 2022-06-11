#!/usr/bin/python3
import rospy 
from ex5.srv import Dodaj, DodajResponse


def handle_add(req): 
	res = DodajResponse() 
	res.suma = req.liczba1 + req.liczba2 
	return res 


def main(): 
	rospy.init_node('service') 
	s = rospy.Service('add', Dodaj, handle_add) 
	rospy.spin() 


if __name__ == "__main__": 
	try: 
		main() 
	except rospy.ROSInterruptException: 
		pass
