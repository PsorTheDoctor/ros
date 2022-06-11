#!/usr/bin/python3
import rospy
from ex4.msg import Student


def main():  
	rospy.init_node('student_publisher')  
	pub = rospy.Publisher('student', Student, queue_size=10)
	rate = rospy.Rate(1)  # once per second  
	
	while not rospy.is_shutdown():   
		rospy.loginfo('Sending data...') 

		student = Student()
		student.numer_albumu = 105155
		student.imie = 'Adam'
		student.nazwisko = 'Wolkowycki'
		student.aktywny = True
		student.oceny = [5.0, 3.0]
		student.kierunek = 'AiR'

		pub.publish(student)    
		rate.sleep()
	
	
if __name__ == '__main__':  
	try:
		main()  
	except rospy.ROSInterruptException as e:
		print(e)
