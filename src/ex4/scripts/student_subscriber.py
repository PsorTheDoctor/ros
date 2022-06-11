#!/usr/bin/python3
import rospy 
# from std_msgs.msg import UInt32, String, Bool 
from ex4.msg import Student


def callback(msg): 
	
	rospy.loginfo('Nr albumu: {}'.format(msg.numer_albumu))
	rospy.loginfo('Imie: {}'.format(msg.imie))
	rospy.loginfo('Nazwisko: {}'.format(msg.nazwisko))
	rospy.loginfo('Aktywny: {}'.format(msg.aktywny))
	rospy.loginfo('Oceny: {}'.format(msg.oceny))
	rospy.loginfo('Kierunek: {}'.format(msg.kierunek))


def main():  
	rospy.init_node('student_subscriber')  
	sub = rospy.Subscriber('student', Student, callback)  
	rospy.spin()

	
if __name__ == '__main__':  
	try:    
		main()  
	except rospy.ROSInterruptException as e:
		print(e)
