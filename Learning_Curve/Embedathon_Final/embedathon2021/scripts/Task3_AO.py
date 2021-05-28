#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import atan2,pow,sqrt,sin,cos,e,pi

x = y = theta = 0.0

regions =  {
            'right': 1,
            'fright': 1,
            'front': 1,
            'fleft': 1,
            'left': 1
           }

def odom_callback(data):
    global x, y, theta
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q = data.pose.pose.orientation
    (_,_,theta) = euler_from_quaternion([q.x,q.y,q.z,q.w])

def laser_callback(msg):
    global regions
    regions = {
                'right':  min(min(msg.ranges[0:72]), 2.1),
                'fright': min(min(msg.ranges[73:144]), 2.1),
                'front':  min(min(msg.ranges[145:216]), 2.1),
                'fleft':  min(min(msg.ranges[217:288]), 2.1),
                'left':   min(min(msg.ranges[289:360]), 2.1)
              }

def controller():
    global x, y, theta, regions
    goal = Point()
    goal.x = 0.0
    goal.y = 0.0
    rospy.init_node('obstacle_avoidance' , anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, laser_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(4)
    velocity = Twist()

    # Moving along the equation

    while not rospy.is_shutdown() and x>-9.5:
        goal.x = x-0.5
        goal.y = sin(2*(goal.x))*sin((goal.x)/2)*(e**-0.01)
        e_x = goal.x - x
        e_y = goal.y - y
        theta_desired = atan2(e_y,e_x)
        e_theta = theta_desired - theta
        e_theta = atan2(sin(e_theta),cos(e_theta))
        if abs(e_theta) > 0.1:
            velocity.linear.x = 0.0
            if (e_theta) > 0.0:
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
    rate.sleep()

    #Avoiding obstacles

    goal.x = -25.0
    goal.y = 0.0
    while not rospy.is_shutdown() and (sqrt((goal.x - x)**2 + (goal.y - y)**2)  >= 0.1):
        if regions['front']>1.5 and regions['fright']>1.5 and regions['fleft']>1.5:
            e_x = goal.x - x
            e_y = goal.y - y
            theta_desired = atan2(e_y,e_x)
            e_theta = theta_desired - theta
            e_theta = atan2(sin(e_theta),cos(e_theta))
            if abs(e_theta) > 0.1:
                velocity.linear.x = 0.0
                if (e_theta) > 0.0:
                    velocity.angular.z = 0.3
                else:
                    velocity.angular.z = -0.3
            else:
                velocity.linear.x = 0.3
                velocity.angular.z = 0.0
        else:
            if regions['front'] > 1.5:
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
            else:
                if regions['fleft'] > regions['fright']:
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.3
                else:
                    velocity.linear.x = 0.0
                    velocity.angular.z = -0.3


        pub.publish(velocity)
        rate.sleep()
    
    print("Reached desired goal (-25, 0)")
    #Avoiding obstacles

    goal.x = -9.5
    goal.y = 0.0
    while (sqrt((goal.x - x)**2 + (goal.y - y)**2)  >= 0.1):        
        if regions['front']>1.5 and regions['fright']>1.5 and regions['fleft']>1.5:
            e_x = goal.x - x
            e_y = goal.y - y
            theta_desired = atan2(e_y,e_x)
            e_theta = theta_desired - theta
            print(e_theta)
            if abs(e_theta) > 0.1:
                velocity.linear.x = 0.0
                if (e_theta) > 0.0:
                    velocity.angular.z = 0.3
                else:
                    velocity.angular.z = -0.3
            else:
                velocity.linear.x = 0.3
                velocity.angular.z = 0.0
        else:
            if regions['front'] > 1.5:
                velocity.linear.x = 0.4
                velocity.angular.z = 0.0
            else:
                if regions['fleft'] > regions['fright']:
                    velocity.linear.x = 0.0
                    velocity.angular.z = 0.3
                else:
                    velocity.linear.x = 0.0
                    velocity.angular.z = -0.3
        pub.publish(velocity)
        rate.sleep()

    #Moving back in equation

    goal.x = 0.0
    goal.y   =  0.0
    while  not rospy.is_shutdown() and (sqrt((goal.x - x)**2 + (goal.y - y)**2)  >= 0.1):
        goal.x = x + 0.4
        goal.y = (sin(2 *(goal.x)) * sin((goal.x)/2)) * (e**-0.01)
        e_y = goal.y - y
        e_x = goal.x - x
        
        theta_desired = atan2(e_y,e_x)
        if abs(theta_desired - theta) >  0.1:
            velocity.linear.x  = 0.0
            velocity.linear.y = 0.0
            if theta_desired  - theta > 0:
                velocity.angular.z = 0.4
            else:
                velocity.angular.z = -0.4

        else:
            velocity.linear.x = 0.4
            velocity.linear.y = 0.0
            velocity.angular.z = 0.0
        
        pub.publish(velocity)
        rate.sleep()

    print("Back to initial position")

    velocity.linear.x = 0.0
    velocity.angular.y = 0.0
    velocity.angular.z  = 0.0
    pub.publish(velocity)

if __name__=='__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass