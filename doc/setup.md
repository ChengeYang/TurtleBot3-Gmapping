# TurtleBot Setup
#### TurtleBot 3 Waffle Pi Model B+

## 1. Hardware Setup
Please follow steps in the User Manual that comes with the TurtleBot.

## 2. RaspberryPi Setup

### 2.1. Ubuntu MATE
My TurtleBot model comes with RaspberryPi 3 Model B+, which is not officially supported by OpenRobotics for Ubuntu MATE. However, the newest release Ubuntu MATE 18.04.2 is available and can be installed on this RaspberryPi model. Please download the OS image from [Ubuntu MATE website](https://ubuntu-mate.org/raspberry-pi/) and follow the steps to restore the image on SD card.

### 2.2. ROS
The Ubuntu MATE 18.04.2 supports ROS Melodic. To install, go to [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu) and follow the steps.

### 2.3. TurtlBot Packages
* Create catkin workspace:
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```
* Download packages from github:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws
$ catkin_make -j1
```

### 2.4. USB Settings
The following commands allow to use USB port for OpenCR without acquiring root permission:
```
$ rosrun turtlebot3_bringup create_udev_rules
```

### 2.5. Network Configuration

#### 2.5.1. Network Setup
* Connect PC and TurtleBot to the same Wifi
* Enter the below command on the terminal window of the TurtleBot PC to find out the IP address of TurtleBot:
```
$ ifconfig
```
* On PC, enter the following command to test the nettwork connection:
```
$ ping <IP_of_TurtleBot>
```

#### 2.5.2. ROS Master Setup
* Enter the following command on TurtleBot:
```
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_HOSTNAME=<IP_of_TurtleBot>
```
* Enter the following command on PC:
```
$ export ROS_MASTER_URI=http://<IP_of_TurtleBot>:11311
$ export ROS_HOSTNAME=<IP_of_PC>
```
Then you can connect to the TurtleBot ROS Master from your PC.

#### 2.5.3. SSH Setup
* Install ssh on both TurtleBot and PC:
```
$ sudo apt update
$ sudo apt install openssh-server
```
* Enable ssh:
```
$ sudo service ssh restart
```
* Regenerate OpenSSH host keys:
```
$ sudo dpkg-reconfigure openssh-server
```
* Check the status of ssh:
```
$ sudo service ssh status
```
Then you can ssh into TurtleBot from your PC using:
```
$ ssh ssh <Username_of_TurtleBot>@<IP_of_TurtleBot>
```

## 3. OpenCR Setup
Enter the following commands in TurtleBot:
```
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=waffle
$ rm -rf ./opencr_update.tar.bz2
$ wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS1/latest/opencr_update.tar.bz2 && tar -xvf opencr_update.tar.bz2 && cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
```

## 4. Basic Operation
To test if the installation is successful, run the following ROS packages to drive the TurtleBot using keyboard:
```
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
$ rosrun turtlebot3_teleop turtlebot3_teleop_key
```
