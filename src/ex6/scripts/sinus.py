#!/usr/bin/python3
import rospy 
from std_msgs.msg import Float32
import math 

def main(): 
	rospy.init_node("params") 
	A = 1.0 
	if rospy.has_param('A'): 
		A = rospy.get_param('A') 
	else: 
		rospy.set_param('A', A) 
		
	w = rospy.get_param('w', 10.0) 
	pub = rospy.Publisher('y', Float32, queue_size=10) 
	t0 = rospy.Time.now()

	rate = rospy.Rate(10)
	while not rospy.is_shutdown(): 
		t = (rospy.Time.now() - t0).to_sec() 
		y = A * math.sin(w * t) 
		msg = Float32() 
		msg.data = y 
		pub.publish(msg) 
		rate.sleep() 
	
if __name__ == "__main__": 
	try: 
		main() 
	except rospy.ROSInterruptException: 
		pass
