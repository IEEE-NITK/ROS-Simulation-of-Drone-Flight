#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def spiral():
    rospy.init_node('circle_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    velocity = Twist()
    rate = rospy.Rate(10) 
    rospy.loginfo("Moving the bot")
    radius = 1

    while not rospy.is_shutdown():
        velocity.linear.x = 1.0
        velocity.angular.z = 1.0/radius
        radius += 0.01
        pub.publish(velocity)
        rate.sleep()
    
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub.publish(velocity)

if __name__=='__main__':
    try:
        spiral()
    except rospy.ROSInterruptException:
        pass