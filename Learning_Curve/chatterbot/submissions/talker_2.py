#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    n= 1
    while(n<101 and rospy.is_shutdown() is not True):
        hello_str = "hello world - %i" % n
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        n+= 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass