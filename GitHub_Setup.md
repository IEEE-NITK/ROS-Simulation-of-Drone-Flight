# Setting up GitHub and Project

Alright team, so far we have almost set up all the prerequisites - Installing Ubuntu and ROS. Now lets finally setup our project onto our systems. As discussed earlier, we will be using GitHub throughout the entirety of the project. Lets go step by step to setting up everything.


## Step 1 - Cloning the Repository

Obviously first step will be to cloning the repository. Go to your workspace and in your src folder, clone the repository into your folder

```bash
cd ros_ws/src
git clone https://github.com/IEEE-NITK/ROS-Simulation-of-Mobile-Robot.git
```

## Step 2 - Change Readme File

Now that you have the contents of the repository, time to change things to your repository. Lets start with naming yourself in the Readme File. Head over to the folder and open the folder in visual studio code

```scala
cd ROS-Simulation-of-Mobile-Robot/
code .
```
 Now in your Readme File change your line 1 to - 
```
1	# This Branch Belongs to Name
```
Save your changes

## Step 3 - Add new Branch and Push

Now make sure you have saved your changes. To make sure the change has been registered. Type this - 

```bash
git status
```
This line should tell you that the last modified file was Readme. 

### Step 3.1 Configuring your GitHub on terminal

To setup your User-ID on Ubuntu terminal, type the following commands along with your credential (Email and GitHub User ID
```bash
git config --global user.email "abc@xyz.com"
git config --global user.name "abc"
```
Make sure you set the correct details as it will cause problems later otherwise

### Step 3.2 Creating a new branch

Every student member will be using a different branch to work with their project. To create a new branch type the following into your terminal - (Make sure to replace your name with name)

```bash
git checkout -b name
```

### Step 3.3 Add the contents to your commit and push

Add your changes and commit changes

```bash
git add .
git commit -m "Created My Branch"
```
Now push your branch, run this command

```bash
git push origin name_of_branch
```

### Viola! You have successfully created your own branch and we are ready to start our project