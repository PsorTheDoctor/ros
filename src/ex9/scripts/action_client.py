#!/usr/bin/python3
import rospy
import actionlib
import ex9.msg


def feedback_cb(msg):
    rospy.loginfo("Feedback: {}".format(msg.feedback.sequence))


def main():
    rospy.init_node("count_client")
    client = actionlib.SimpleActionClient("count", ex9.msg.CountAction)
    sub = rospy.Subscriber("/count/feedback", ex9.msg.CountActionFeedback, feedback_cb)
    client.wait_for_server()

    target = ex9.msg.CountGoal()
    target.value = 10
    target.step = int(input('Enter incrementation step: '))

    rospy.loginfo("Sending target: {}".format(target.value))
    client.send_goal(target)
    client.wait_for_result()
    result = client.get_result()
    print(result)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
