#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    n = 1
    while not rospy.is_shutdown() and n <= 100:
        hello_str = "Hello world - " + str(n)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
        n += 1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass