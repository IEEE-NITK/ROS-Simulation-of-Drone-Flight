# Installing Ubuntu 18.04 in Virtual Box!

## Step 1 - Downloading Necessary Items   

 - [ ] Download ISO file for Ubuntu 18.04 Bionic Beaver. [(link)](https://releases.ubuntu.com/18.04/ubuntu-18.04.5-desktop-amd64.iso)
 - [ ] Download Virtual box. [(link)](https://www.virtualbox.org/)

## Step 2 - Setup and Allocation of Storage in Virtual Box   

 - [ ] Open Virtual Box and Select New from the top Ribbon. Enter your desired name for your image (can be anything). Select Type - **Linux** and Version - **Ubuntu_64**
 - [ ] Allocate Memory Size for your Virtual Machine. This will be your RAM that will account for usage in your image. 
 > Preferably pull the slider till the end of the green section but otherwise **4 GB** will be sufficient. 
 - [ ] Select Create a Virtual Hard Disk Now. Select **VDI (VirtualBox Disk Image)** on next popup. **Dynamically Allocated** on next. 
 >	This will allow Virtual Box to store data on a location in your Hard disk but not completely as it will grow every time new data is written to for the first time, until the drive reaches the maximum capacity chosen when the drive was created.
 - [ ] Allocate a **minimum of 10 GB** to your drive. Again, this will be tentative and dynamic and not permanent so you do not have to worry about less storage for your current operating system.
 - [ ] Save and Apply. 

##  Step 3 - Installing Ubuntu 18-04

 - [ ] Select the newly made Virtual machine and select Start from the top Ribbon
 - [ ] On the Next Pop-up, click on the folder icon, click on "Add" and browse for the Ubuntu 18.04 ISO file we downloaded on Step 1. Click Choose and Press Start.
 - [ ] Follow the basic guidelines keeping everything as recommended for Ubuntu Installation. 
 - [ ] Under Updates and Software's Section while installing, do **select** "*Install third-party software for Graphics and Wi-Fi hardware and additional media formats*"
 - [ ] Under Installation Type Menu, **Select** *Erase Disk and install Ubuntu*. 
>  None of your current operating files will be affected. Rest Assured!
 - [ ] CONGRATULATIONS! You have Successfully installed Ubuntu 18.04 as your Virtual Machine!

 ##  Step 3 - Before we get started
 > Here are some tips and tricks that will help you get a better experience in Ubuntu

 - [ ] Keeping your Clipboard Bi-directional from Windows to Virtual Machine and Vice-verse.
 > Shut Down your Ubuntu and in your Virtual Box Ribbon go to Settings -> General -> Advanced. Select Bidirectional for "Shared Clipboard" and "Drag'n'Drop". Press OK.

 - [ ] For getting Full Screen of your Ubuntu Virtual Machine
> Start your Ubuntu Image. Under your Devices Panel select "Insert Guest Additions CD Image". Select Run and enter your credentials. Let the terminal run. Once you get the confirmation message regarding completion of task. Press Enter and restart your Virtual Box. 