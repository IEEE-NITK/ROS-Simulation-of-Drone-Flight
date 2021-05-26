#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2,pow,sqrt,sin,cos,e,pi

x = y = theta = 0.0

def odom_callback(data):
    global x, y, theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    (_,_,theta) = euler_from_quaternion([q.x,q.y,q.z,q.w])
    
def controller():
    global x, y, theta
    goal = Point()
    goal.x = 0.0
    goal.y = 0.0
    rospy.init_node('obstacle_avoidance' , anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(4)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    velocity = Twist()
    rospy.loginfo("MOVING ALONG THE EQUATION")
    
    while not rospy.is_shutdown() and x<18.5:
        goal.x = x+(2*pi)/10
        goal.y = sin(2*(goal.x-9))*sin((goal.x-9)/2)*pow(e , -0.01)
        e_x = goal.x - x
        e_y = goal.y - y
        theta_desired = atan2(e_y,e_x)
        if abs(theta_desired - theta) > 0.1:
            velocity.linear.x = 0.0
            if (theta_desired -theta) > 0.0:
                velocity.angular.z = 0.3
            else:
                velocity.angular.z = -0.3
        else:
            velocity.linear.x = 0.3
            velocity.angular.z = 0.0
        pub.publish(velocity)
        rate.sleep()

    velocity.linear.x = 0.0
    velocity.angular.z = 0.0
    pub.publish(velocity)

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass