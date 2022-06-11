#!/usr/bin/python3
import rospy 
from std_msgs.msg import Float32
import math 

def pid(measured_value, kp=1.0, ki=1.0, kd=1.0): 
	
	rospy.init_node('pid') 
	kp = rospy.get_param('Kp', kp)
	ki = rospy.get_param('Ki', ki)
	kd = rospy.get_param('Kd', kd)
	dt = 0.1
	
	prev_error = 0.0
	integral = 0.0
	setpoint = 1.0
	output = -1.0

	rate = rospy.Rate(1.0 / dt)
	while not rospy.is_shutdown(): 
		
		error = setpoint - measured_value
		proportional = error
		integral += error * dt
		derivative = (error - prev_error) / dt
		
		output = kp * proportional + ki * integral + kd * derivative
		prev_error = error
		rate.sleep()
		
		rospy.loginfo('Output={} Error={}'.format(output, error))
		
	return output
	
def main():
	pub = rospy.Publisher('y', Float32, queue_size=10) 
	
	kp = float(input('Enter Kp: '))
	ki = float(input('Enter Ki: '))
	kd = float(input('Enter Kd: '))
	
	y = pid(5.0, kp, ki, kd)
	msg = Float32() 
	msg.data = y 
	pub.publish(msg) 
	
if __name__ == "__main__": 
	try: 
		main() 
	except rospy.ROSInterruptException: 
		pass
