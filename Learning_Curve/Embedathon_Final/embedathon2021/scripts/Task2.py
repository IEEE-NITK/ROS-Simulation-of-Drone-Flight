#!/usr/bin/env python2.7

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2, sin, cos, sqrt, e

x = y = theta = 0.0

def callback(data):
    global x, y, theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    (_, _, theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

def move_bot():
    global x, y, theta
    
    rospy.init_node('go_to_goal', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)
    vel = Twist()
    
    rate = rospy.Rate(10)
    
    x1 = 0.0
    y1   =  0.0
    while  not rospy.is_shutdown() and x1 <= 19.00:
        x1 = x + 0.4
        y1 = (sin(2 *(x1  - 9)) * sin((x1 - 9)/2)) * (e**-0.01)
        e_y = y1 - y
        e_x = x1 - x
        
        theta_desired = atan2(e_y,e_x)
        if abs(theta_desired - theta) >  0.1:
            vel.linear.x  = 0.0
            vel.linear.y = 0.0
            if theta_desired  - theta > 0:
                vel.angular.z = 0.4
            else:
                vel.angular.z = -0.4

        else:
            vel.linear.x = 0.4
            vel.linear.y = 0.0
            vel.angular.z = 0.0
        
        pub.publish(vel)
        rate.sleep()
    vel.linear.x = 0.0
    vel.angular.y = 0.0
    vel.angular.z  = 0.0
    pub.publish(vel)

if __name__=='__main__':
    try:
        move_bot()
    except rospy.ROSInterruptException:
        pass