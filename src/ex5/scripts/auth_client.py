#!/usr/bin/python3
import rospy
from ex5.srv import Auth


def main():
    rospy.init_node('client')
    rospy.wait_for_service('auth')
    login = input('Enter your login: ')
    password = input('Type your password: ')
    try:
        auth_proxy = rospy.ServiceProxy('auth', Auth)
        res = auth_proxy(login, password)
        if res.success:
            rospy.loginfo(rospy.loginfo('Successfully logged in!'))
        else:
            rospy.loginfo(rospy.loginfo('Login failed! Try again.'))

    except rospy.ServiceException as e:
        print('Service error: %s' % e)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
