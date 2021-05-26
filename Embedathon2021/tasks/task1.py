#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def circle():
    rospy.init_node('circle_node' , anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    velocity = Twist()
    rate = rospy.Rate(10)
    rospy.loginfo("Moving in a circle")
    radius=3.5

    while not rospy.is_shutdown():
        velocity.linear.x = 0.5
        velocity.angular.z = 0.5/radius
        pub.publish(velocity)
        rate.sleep()

    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    pub.publish(velocity)

if __name__ == '__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass