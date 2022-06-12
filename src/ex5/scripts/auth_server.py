#!/usr/bin/python3
import rospy
from ex5.srv import Auth, AuthResponse

login = 'user'
password = 'secret'


def handle_auth(req):
    global login, password
    res = AuthResponse()
    res.success = req.login == login and req.password == password
    return res


def main():
    rospy.init_node('service')
    global login, password
    service = rospy.Service('auth', Auth, handle_auth)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
