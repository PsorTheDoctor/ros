#!/usr/bin/python3
import rospy
import actionlib
import ex9.msg
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

turtleX = 0
turtleY = 0
turtleAngle = 0


def pose_cb(pos):
    global turtleX, turtleY, turtleAngle
    turtleX = pos.x
    turtleY = pos.y
    turtleAngle = pos.theta


class TurtleServer:
    def __init__(self, name):
        self._name = name
        self._as = actionlib.SimpleActionServer(self._name, ex9.msg.TurtleAction,
                                                execute_cb=self.turtle_cb, auto_start=False)
        self._as.start()

    def turtle_cb(self, target):
        rospy.loginfo('Desired location: x={} y={}'.format(target.x, target.y))
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber('/turtle1/pose', Pose, pose_cb)

        feedback = ex9.msg.TurtleFeedback()
        dist = math.sqrt((turtleX - target.x) ** 2 + (turtleY - target.y) ** 2)
        feedback.distance.append(dist)
        # result = ex9.msg.TurtleResult()

        threshold = 0.5
        twist = Twist()
        while feedback.distance[len(feedback.distance) - 1] > threshold:
            angleDiff = math.atan2(target.y - turtleY, target.x - turtleX)
            angleFinal = angleDiff - turtleAngle

            if (turtleX < (target.x - threshold) or turtleX > (target.x + threshold)) and \
                (turtleY < (target.y - threshold) or turtleY > (target.y + threshold)):

                if abs(angleFinal) > 0.1:
                    twist.linear.x = 0.0
                    twist.angular.z = angleDiff * 2
                else:
                    twist.angular.z = 0.0
                    twist.linear.x = 2.0
            else:
                twist.angular.z = 0.0
                twist.linear.x = 2.0

            pub.publish(twist)
            rospy.spin()
            rate = rospy.Rate(1)
            success = True

            if self._as.is_preempt_requested():
                rospy.loginfo('Early stopping')
                self._as.set_preempted()
                success = False
                break

            rospy.loginfo('Turtle is crawling at x={} y={}'.format(turtleX, turtleY))
            dist = math.sqrt((turtleX - target.x) ** 2 + (turtleY - target.y) ** 2)
            feedback.distance.append(dist)
            self._as.publish_feedback(feedback)
            rate.sleep()
            if success:
                result = ex9.msg.TurtleResult()
                self._as.set_succeeded(result)


def main():
    rospy.init_node('turtle_server')
    TurtleServer('turtle')
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
