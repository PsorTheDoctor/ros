#!/usr/bin/python3
import rospy 
from std_msgs.msg import Int32, String


def main(): 
    rospy.init_node('my_publisher')

    pub = rospy.Publisher('text', String, queue_size=10)
    text = 'sth'
    number = 0
    rate = rospy.Rate(2)  # twice per second

    while not rospy.is_shutdown():

        rospy.loginfo("Sending text: " + text + str(number))
        msg = String()
        msg.data = text + str(number)
        pub.publish(msg)

        number += 1
        rate.sleep()

 
if __name__ == '__main__': 
  try: 
      main()
  except rospy.ROSInterruptException: 
      pass
