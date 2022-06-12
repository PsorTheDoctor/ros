#!/usr/bin/python3
import rospy
from ex5.srv import Sinus


def main():
    rospy.init_node('client')
    rospy.wait_for_service('sinus')

    while not rospy.is_shutdown():
        A = float(input('A = '))
        w = float(input('w = '))
        p = float(input('w = '))
        try:
            sinus_proxy = rospy.ServiceProxy('sinus', Sinus)
            res = sinus_proxy(A, w, p)
            rospy.loginfo('y = {}'.format(res.y))

        except rospy.ServiceException as e:
            print("Service error: %s" % e)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
