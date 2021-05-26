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

def laser_callback(l_data):
    global regions
    regions = {
                'right':  min(min(l_data.ranges[0:72]), 2.1),
                'fright': min(min(l_data.ranges[73:144]), 2.1),
                'front':  min(min(l_data.ranges[145:216]), 2.1),
                'fleft':  min(min(l_data.ranges[217:288]), 2.1),
                'left':   min(min(l_data.ranges[289:360]), 2.1)
              }

goal = Point()
goal.x = 0.0
goal.y = 0.0
rospy.init_node('obstacle_avoidance' , anonymous=True)
rospy.Subscriber('/odom', Odometry, odom_callback)
rospy.Subscriber('/scan', LaserScan, laser_callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(5)
velocity = Twist()

# Moving along the equation

rospy.loginfo("MOVING ALONG EQUATION")
while not rospy.is_shutdown() and x>-9.5:
    goal.x = x-0.5
    goal.y = sin(2*(goal.x))*sin((goal.x)/2)*pow(e , -0.01)
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

# Avoiding obstacles

rospy.loginfo("MOVING TO GOAL AVOIDING OBSTACLES")
goal.x = -25.0
goal.y = 0
while not rospy.is_shutdown() and x>=-25.0:
    if regions['front']>1.8 and regions['fright']>1.6 and regions['fleft']>1.6 and regions['right']>0.5 and regions['left']>0.5:
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
        if regions['front']>1.5:
            velocity.linear.x = 0.3
            velocity.angular.z = 0.0
        else:
            if regions['fleft']>regions['fright']:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.3
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = -0.3
    pub.publish(velocity)
    rate.sleep()
velocity.linear.x = 0.0
velocity.angular.z = 0.0
pub.publish(velocity)
rate.sleep()

# Moving back avoiding obstacles

rospy.loginfo("REACHED THE GOAL, NOW MOVING BACK AVOIDING OBSTACLES")
goal.x = -9.5
goal.y = 0
while not rospy.is_shutdown() and x<-9.5:
    if regions['front']>1.8 and regions['fright']>1.6 and regions['fleft']>1.6 and regions['right']>0.5 and regions['left']>0.5:
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
        if regions['front']>1.5:
            velocity.linear.x = 0.3
            velocity.angular.z = 0.0
        else:
            if regions['fleft']>regions['fright']:
                velocity.linear.x = 0.0
                velocity.angular.z = 0.3
            else:
                velocity.linear.x = 0.0
                velocity.angular.z = -0.3
    pub.publish(velocity)
    rate.sleep()
velocity.linear.x = 0.0
velocity.angular.z = 0.0
pub.publish(velocity)
rate.sleep()

# Moving back along the equation

rospy.loginfo("MOVING BACK ALONG EQUATION")
while not rospy.is_shutdown() and x<=0:
    goal.x = x+0.5
    goal.y = sin(2*(goal.x))*sin((goal.x)/2)*pow(e , -0.01)
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