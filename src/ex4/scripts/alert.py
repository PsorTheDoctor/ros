#!/usr/bin/python3
import rospy
import math
from std_msgs.msg import Float32
from ex4.msg import Alert

msg = Float32()
alert = Alert()

upperLimit = 80
lowerLimit = 20
temp = 0.0
t0 = 0.0


def callback(msg):
    global alert, temp, t0
    temp = msg.data
    rospy.loginfo('Current temperature: {}'.format(temp))
    if not alert.state:
        t0 = rospy.Time.now().to_sec()


def main():
    rospy.init_node('furnace')
    pub = rospy.Publisher('temperature', Float32, queue_size=10)
    pub2 = rospy.Publisher('alert', Alert, queue_size=10)
    sub = rospy.Subscriber('temperature', Float32, callback)
    rate = rospy.Rate(10)

    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo('Current temperature: {}'.format(temp))
        i += 1
        rad = i * math.pi / 180
        signal = 50 + math.sin(rad) * 50
        msg.data = signal

        if temp < lowerLimit:
            rospy.loginfo('ALERT: Lower limit!')
            alert.limit = 'lower limit'
            alert.temp_diff = lowerLimit - temp
            alert.time = rospy.Time.now().to_sec() - t0
            alert.state = True

        elif temp > upperLimit:
            rospy.loginfo('ALERT: Upper limit!')
            alert.limit = 'upper limit'
            alert.temp_diff = temp - upperLimit
            alert.time = rospy.Time.now().to_sec() - t0
            alert.state = True
        else:
            alert.state = False
            alert.limit = ''
            alert.temp_diff = 0.0
            alert.time = 0.0

        pub.publish(msg)
        pub2.publish(alert)
        # rospy.spin()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException as e:
        print(e)
