#!/usr/bin/python3
import rospy
import actionlib
import ex9.msg


class CountServer:
	def __init__(self, name):
		self._name = name         
		self._as = actionlib.SimpleActionServer(self._name, ex9.msg.CountAction,
												execute_cb=self.count_cb, auto_start=False)
		self._as.start()  
		
	def count_cb(self, target):
		rospy.loginfo("Received target: {}".format(target.value))
		x = 0    
		feedback = ex9.msg.CountFeedback()
		feedback.sequence.append(x)
		result = ex9.msg.CountResult()
		rate = rospy.Rate(1)    
		success = True

		while x < target.value:
			if self._as.is_preempt_requested():
				rospy.loginfo('Early stopping')
				self._as.set_preempted()
				success = False
				break

			x += target.step
			rospy.loginfo("Counting %d", x)
			feedback.sequence.append(x)
			self._as.publish_feedback(feedback)
			rate.sleep()
			if success:
				result = feedback
				self._as.set_succeeded(result)


def main():
	rospy.init_node("count_server")
	CountServer('count')
	rospy.spin()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
