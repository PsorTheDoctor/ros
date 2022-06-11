#!/usr/bin/python3
import rospy 
from std_msgs.msg import Int32, String


def text_callback(msg):
    text = msg.data
    rospy.loginfo("Received text: " + text)


def main():
    rospy.init_node('my_subscriber')
    sub = rospy.Subscriber('text', String, text_callback)
    rospy.spin()


if __name__ == '__main__': 
  try: 
      main()
  except rospy.ROSInterruptException: 
      pass
