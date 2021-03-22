# MiniProject - 1: Chatterbot

Alright gang, lets start with our first project in ROS. We'll explain you everything about the functionality of the ROS and how different aspects of it work together to eventually build a robot environment we are looking for. Over this project we'll accomplish two projects (sort of) :-
1. Hello World
2. Chatterbot

So, lets begin!
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

## Step 2 - Creating your first Package

Its about time we started working with packages on our systems. To do so, first make you have sources your setup correctly.

```bash
source ~/ros_ws/devel/setup.bash
```

*Now, make sure you are in the correct directory, "ros_ws/src/ROS-Simulation-of-Drone-Flight/Learning_Curve/*"

To create a package, run the following command:
```bash
catkin_create_pkg Chatterbot
```
Finally, compile your workspace
```bash
catkin build
```

Viola! You have setup your first package!

## Step 3 - Creating Projects
Now lets get to business. Time to start coding and seeing ROS do its work. We'll start with a simple Hello World Code and move on to creating a Chatterbot.

Navigate to your package and make a new folder named "scripts". Navigate inside
```bash
cd Chatterbot
mkdir scripts
cd scripts
```
### Step 3.1 - Hello World!

Lets open Code, it'll help us as we go, 
```bash
code .
```
Lets create a python file for this, you can do by using  visual studio code or by terminal also, type this onto your terminal-
```bash
touch hello_world.py
```
Now in your python file, copy paste this code - 

```python
#!/usr/bin/env python

import rospy

def main():    
    
    # 1. Make the script a ROS Node.
    rospy.init_node('node_hello_ros', anonymous=True)

    # 2. Print info on console.
    rospy.loginfo("Hello World!")
    
    # 3. Keep the node alive till it is killed by the user.
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```
Lets go line by line - 
```python
#!/usr/bin/env python
```
This line will set the interpreter to python2.7 which ROS uses. If you do not do this, it'll cause a lot of errors as there will be conflicts from 2.7 and 3 version. This is called "shebang"
```python
import rospy
```
This will import rospy library to your script allowing you to create nodes, publishers, subscribers etc. 
```python
rospy.init_node('node_hello_ros', anonymous=True)
```
Everything in ROS Works using "nodes" which can be thought of robots or clients in a server. They can communicate over a server "roscore". So we run the above command to create a node. You cannot communicate in ROS without being a node.

"anonymous = True" ensures that your node has a unique name by adding random numbers to the end of the name of your node ('node_hello_ros' in our case). 
```python
rospy.loginfo("Hello World!")
```
This line will just print a "Hello World" into your terminal by sending it to /rosout rostopic. 

```bash
rospy.spin()
```
Keep printing till we do not stop it using "Ctrl+C".

#### Running a Python Script

Now lets run this python file. To do so, we'll have to make it executable. To check if its executable, run:
```bash
ls
```
You will see that "hello_world.py" is in white. An executable file will be shown in green.
To make it executable, run the following command:

```bash
sudo chmod +x hello_world.py
```
Enter your password and type "ls" again, you will see that the file is in green.
Now that our file is executable, time to run it. To run a rosnode, we'll have to start the master server (roscore). To do so, run:
```bash
roscore
```
Now open a new terminal and type -
```bash
rosrun Chatterbot hello_world.py
```
You should see a "Hello World!" bring printed. 
Congratulations XD

### Step 3.2 - Chatterbot

A chatterbot in this project is essentially a publisher(talker) and a subscriber(listener) talking to each other. It's used for robot-sensor communication where the sensor asks a publisher - publishing content (known as ROS Messages) to a robot which acts as a subscriber.

We'll first code a publisher (talker) and then create a subscriber (listener). Lets gooooooooo.

#### Step 3.2.1 - Talker

Lets start by creating a Python script called "talker"

```bash
touch talker.py
```
Open the file in code and type the following lines

```python 
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

```
Lets again go line by line - 

```python
import rospy
from std_msgs.msg import String
```
As mentioned earlier, publisher and subscriber communicates using ROS Messages. The std_msgs.msg import is so that we can use the std_msgs/String ROS message type (a simple string container) for publishing. 

```python
pub = rospy.Publisher('chatter', String, queue_size=10)
```

This declares that your node is publishing to the "chatter" topic using the message type String which we imported earlier. Think of a topic as your railway lines and your messages as your train, you will understand that you need a topic to send a message. 

```python
rospy.init_node('talker', anonymous=True)
```
Again, lets initialise a node for communication
``` python
rate = rospy.Rate(10) 
```
This line creates a Rate object "rate" and setting its value as 10. With its argument of 10, we should expect to go through the loop 10 times per second or in our case, send our message 10 times in a second.

```python
while not rospy.is_shutdown():
    hello_str = "hello world %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    rate.sleep()
```
This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. It is used a lot in ROS Applications if you want your loop to run for the entirety of the duration of the execution. 
We then create a string containing the content along with the time for uniqueness. We then print our message on the terminal using "loginfo". 

Finally, we publish our message to the topic using pub.publish(). This will send our string to "chatter" topic as we initialised earlier. Finally rate.sleep() is used so that we send only 10 messages (as we initialised earlier) and not go on an infinite loop.

Well... Now its time to create a subscriber which listens to this topic.

#### Step 3.2.2 - Listener

Create a Python script called listener.

``` bash
touch listener.py
```
Open  code and Add the following code in the python file:

``` python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```
Again, lets go step by step
``` python
rospy.init_node('listener', anonymous=True)
```
We obviously create a node first. All nodes should have different names.
```bash
rospy.Subscriber("chatter", String, callback)
```

 This declares that your node subscribes to the chatter topic which is of type std_msgs.msgs.String. When new messages are received, callback is invoked with the message as the first argument.
```bash
rospy.spin()
```
This line will keep the entire python file in execution till we do not kill it ourselves (loyal af).
AND WE ARE DONE!

#### Running the files

Follow the same steps as we did earlier to make the two files executable. 
Now in one terminal, run:
```bash
roscore
```
Open another terminal and run - 
```bash
rosrun Chatterbot listener.py
```
And now run your talker - 
```bash
rosrun Chatterbot talker.py
```
You should be able to see them both communicating over.

## Ha... Assignment here

Now that you know how to work around with publisher and subscribers, let's put your brains to good use. You have to achieve the desired tasks - 

 1. Create two python files - "talker_2.py" and "listener_2.py".  
 2. The talker should just to the chatter messages like:
	> Hello World - 1  
	> Hello World - 2  
	> Hello World - 3  
	>.  
	>.  
	> Hello World - 100  
 3. This should be implemented using a loop in the talker. Only 100 messages should be sent over the topic.
 4. These msgs should be displayed in your lister as:
	 > Heard Hello World - 1  
	 > Heard Hello World - 2  
	 > Heard Hello World - 3  
	 > .  
	 >.  
	 > Heard Hello World - 100  
 5. Make a new folder named "Submission" inside "Chatterbot". 
 6. Make a screen recording of you running both the python script, listener first and then talker and put the video file inside this folder.
 7. Push your code using:
```bash
git push origin (your branch name)
```


