
# Getting Started with ROS Melodic - Installations!

  

## Understanding ROS Distributions

A ROS distribution is a set of ROS packages developed for a specific Ubuntu version. The purpose of the ROS distributions is let programmers and developers work with a stable versions of ROS corresponding to their Ubuntu Version without version conflict errors which occur infamously as you migrate to different Ubuntu versions. There are various ROS distributions such as Melodic (Ubuntu 18.04), Noetic (Ubuntu 20.04), Kinetic(Ubuntu 16.04) etc. Check them out here [here](http://wiki.ros.org/Distributions).

For the entire implementation of this project, we will be going forward with **ROS Melodic**. Why? Because it has been out for quite a while and many new packages have been completely migrated from Kinetic to Melodic. (It takes time so unless you are not a developer, stick to ROS versions which have been around for a while and are still in development cycle)

## Step 1 - Installing ROS

Refer to [this](http://wiki.ros.org/melodic/Installation/Ubuntu) website for proper documentation of all commands to install ROS. For the lazy student members, we have summarized the basic commands flow below. These are to entered in your terminal, you can open it by "Ctrl + T"  - 

Setup your computer to accept software from ROS
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

Update all available packages for your ubuntu (will make ROS accessible for download)
```bash
sudo apt update
```

Install ROS
```bash
sudo apt install ros-melodic-desktop-full
```
Source it to terminal file. You will use this a lot later on when you have multiple workspaces to work on.

```bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Install other dependencies
```bash
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools
```

```bash
sudo apt install python-rosdep
```
Initializes ROS in your computer. You will be able to perform all ROS functions here on from running this command.
```bash
sudo rosdep init
rosdep update
```

## Step 2 - Making your workspace

Now lets get down to making your own workspace. Workspaces are folders where you will be storing, modifying and creating your ROS Packages. Packages for now can be thought of as parts of your software for example in a robot you can have navigation package, perception package, Robot Controller package etc. By combining packages you can create some meaningful robots.

The following terminal commands will setup your ROS Workspace:

**Assuming name of your workspace to be "ros_ws".** Make a directory named "ros_ws" along with an empty folder named "src" inside. This can be done using "mkdir" command. "-p" Argument will allow subfolders to be build in one go as we are creating creating "src" inside "ros_ws"

```bash
mkdir -p ~/ros_ws/src
```
Go to the folder in terminal, can be done by "cd" followed by the path.
```bash
cd ~/ros_ws/
```
Initialize the folder as directory. Most compiling, creating and removing of workspaces and packages is done by "catkin"
```bash
catkin init
```
Build your workspace
```bash
catkin build
```
Source your workspace to be available with your terminal.
```bash
source devel/setup.bash
```

Echo the ROS Package Path to check if everything has been completed smoothly.
```bash
echo $ROS_PACKAGE_PATH
> /home/youruser/ros_ws/src:/opt/ros/melodic/share
```

Viola! You have successfully installed and setup ROS in your computer

> Fun Note - If you check out the ROS Distribution version, you will see nice arts depicting the version. We'll get to seem them soon. XD
