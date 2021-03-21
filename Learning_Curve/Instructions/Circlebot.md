# MiniProject - 2: CircleBot

So now's when we move into earnestly moving onto what will be the main goal of this project. Moving our bots. We will be using the turtlesim which has already been demonstrated to you to simulate our bot moving in a 2D plane, which will later help us do the same thing in a 3D world in Gazebo. Let's start!

## Step 1 - Pull the Branch
Go to your project directory
```bash
cd ros_ws/src/ROS-Simulation-of-Drone-Flight
```
All the projects and setup will be set on the master branch. So to start your work, always first pull the master branch by using the following command - 

```bash
git pull origin main
```

After the pull is completed, you will see a new folder named "Learning_Curve". Inside it will be a folder named "Instructions". Navigate there by using the following commands

```bash
cd Learning_Curve
```
## Step 2 - Run turtlesim
After making sure that your workspace is properly sourced, open up a new terminal and run the following command:
```bash
roscore
```
Open up another terminal (in a new window or a tab) and run the command:
```bash
rosrun turtlesim turtlesim_node
```
Another window with a cute turtle in it will open up, which will be simulating your bot. Here we can take a slight detour. Run the following command in another terminal to be able to control the turtle via the arrow keys on your keyboard:
```bash
rosrun turtlesim turtle_teleop_key
```
Be sure to keep the terminal with the tele-operation running on the top, and voila! you can now move your turtle with your keyboard. Go ahead and close the turtlesim and open up a fresh one, because we are going to need it for the circlebot.  
Now we will check the topics that this turtlesim_node is subscribing to. So open up another terminal and run:
```bash
rostopic list
```
This will list all the topics the node is subscribing to. The most important topic here is */turtle1/cmd_vel* which is the topic through which we publish the velocities to move the bot. We can see information about this particular topic by running:
```bash
rostopic info /turtle1/cmd_vel
```
We see that the topic is of type *geometry_msgs/Twist*. We can also get information on a message like this running:
```bash
rosmsg info geometry_msgs/Twist
```
Here we see that this message contains 3 linear components and 3 angular components of velocity, but we are only concerned with the linear x, linear y, and angular z velocities since this our bot is moving in 2D space. Now moving on the the actual code for moving our bot in a circle.

## Step 3 - Writing the Python Script
Create and build a new package by running the following commands in succession:
```bash
cd ~/ros_ws/src/ROS-Simulation-of-Drone-Flight/Learning_Curve/
catkin_create_pkg circlebot 
catkin build
cd circlebot
mkdir scripts
code .
```
Now down to business, open up your scripts folder in your VS code and create a new file circle_bot.py and paste the following code in your python file:

```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def circle():
    rospy.init_node('circle_node', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    velocity = Twist()
    rate = rospy.Rate(10) #10Hz
    rospy.loginfo("Moving the bot")
    radius = 1

    while not rospy.is_shutdown():
        velocity.linear.x = 1.0
        velocity.angular.z = 1.0/radius
        pub.publish(velocity)
        rate.sleep()
    
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub.publish(velocity)

if __name__=='__main__':
    try:
        circle()
    except rospy.ROSInterruptException:
        pass
```
Now for the explanation, after the shebang line, we first import our required libraries. First rospy (obviously) and then from geometry_msgs we import the message Twist which is the message which will publish the velocities to our bot.
```python
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
```
Here we create three important things within our circle function, first we initialize out node like always. Second, we create a publisher object which publishes to the topic */turtle1/cmd_vel* which we had found out earlier in our terminals with a message type Twist, also about which we already found out. And last, we create an object *velocity* which will be the message to be published.
```python
rospy.init_node('circle_node', anonymous=True)
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
velocity = Twist()
```
Then we define a variable rate which would define how fast messages will be published, in this case at a frequency of 10 Hz or 10 messages per second. 
```python
rate = rospy.Rate(10) #10Hz
```
Then we assign the values of linear velocity and angular velocity, (in relation to the given radius) to our object velocity, and publish the message. After that we stop the messages for 0.1 secs by doing *rate.sleep()* and then the loop continues till a keyboard interrupt is provided.
```python
while not rospy.is_shutdown():
        velocity.linear.x = 1.0
        velocity.angular.z = 1.0/radius
        pub.publish(velocity)
        rate.sleep()
```
After the keyboard interrupt we make the velocities of our turtle to be zero, and publish those velocities to stop our bot.
```python
velocity.linear.x = 0
velocity.angular.z = 0
pub.publish(velocity)
```
## Step 4 - Running the Node
After saving the python file, open a new terminal and run the following to make the file executable:
```bash
cd ~/ros_ws/src/scripts
sudo chmod +x circle_bot.py
```
Now we are ready to run our circle bot node:
```bash
rosrun circlebot circle_bot.py
```
And now if you open up your turtlesim window you will see you turtle moving in circles!!
And now for the interesting part....

## Assignment 2
1. Create another python file in the scripts folder in your circlebot package to move the turtle around in a spiral.
2. Make a new folder named "Submission" inside "circlebot".
3. Make a screen recording of you running the python script and the turtle moving in spiral motions
4. Push you code:
```bash
git push origin (your branch name)
```  