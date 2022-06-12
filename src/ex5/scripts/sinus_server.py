#!/usr/bin/python3
import rospy
import math
from ex5.srv import Sinus, SinusResponse


def handle_change(req):
	res = SinusResponse()
	t = rospy.Time.now().to_sec()
	res.y = req.A * math.sin(req.w * t + req.p)
	return res


def main():
	rospy.init_node('service')
	service = rospy.Service('sinus', Sinus, handle_change)
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
