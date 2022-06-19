#!/usr/bin/python3
import rospy
import actionlib
import ex9.msg


def feedback_cb(msg):
    rospy.loginfo("Feedback: {}".format(msg.feedback.distance))


def main():
    rospy.init_node("turtle_client")
    client = actionlib.SimpleActionClient("turtle", ex9.msg.TurtleAction)
    sub = rospy.Subscriber("/turtle/feedback", ex9.msg.TurtleActionFeedback, feedback_cb)
    client.wait_for_server()

    target = ex9.msg.TurtleGoal()
    target.x = 10.0
    target.y = 10.0

    rospy.loginfo('Sending target: x={} y={}'.format(target.x, target.y))
    client.send_goal(target)
    client.wait_for_result()
    result = client.get_result()
    print(result)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
