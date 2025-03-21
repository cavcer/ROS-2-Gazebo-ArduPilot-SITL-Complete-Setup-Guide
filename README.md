# ROS 2 GAZEBO ARDUPILOT SITL COMPLETE SETUP GUIDE

**video tutoriol at https://www.youtube.com/watch?v=2BhyKyzKAbM&t=1759s**


### Install Git and Curl
Ensure `git` is installed for cloning repositories and `curl` for downloading necessary files.

```bash
sudo apt update
sudo apt install git curl -y
```

### Check and Set UTF-8 Locale
Make sure your system uses UTF-8 encoding to avoid localization issues.

```bash
locale  # Check current locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verify locale settings
```

### Enable Universe Repository
Enable the universe repository to install necessary dependencies.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe # Press Enter
```

## Install ROS 2 Humble

### Add ROS 2 Repository and Key
Download and add the ROS 2 repository key to ensure secure package installations.

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

Add the official ROS 2 package repository to your system.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package lists and upgrade installed packages. 

```bash
sudo apt update
sudo apt upgrade
```
Install the full ROS 2 Humble desktop version, including core libraries, visualization, and simulation tools. This may take a few minutes.

```bash
sudo apt install ros-humble-desktop
```


## Setup Environment

Automatically source ROS 2 setup when opening a terminal.Add the following second line at the end , save and exit:

```bash
gedit ~/.bashrc  # Open .bashrc file
source /opt/ros/humble/setup.bash
source ~/.bashrc # apply the changes
```

## Create a ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Clone example ROS 2 tutorials:

```bash
git clone https://github.com/ros/ros_tutorials.git -b humble
```

## Install Dependencies

### Install Missing Dependencies
Use `rosdep` to install missing dependencies.

```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

If `rosdep` is not found, install it first:

```bash
sudo apt install python3-rosdep2
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

### Install Colcon
Install `colcon` to build ROS 2 packages.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

## Build and Source Workspace

Compile the workspace using `colcon build`.

```bash
colcon build
```

Set up the workspace environment before running ROS 2 nodes.

```bash
source install/local_setup.bash
```

## Test Installation

Run the `turtlesim` node to verify the installation.

```bash
ros2 run turtlesim turtlesim_node
```

If `turtlesim` launches successfully, your ROS 2 installation is complete!

## SOURCE ROS2 ENVİRONMENT AND CHECK
Check that variables like ROS_DISTRO and ROS_VERSION are set.
```bash
printenv | grep -i ROS
```
Open the .bashrc file and add these two export lines at the end.
```bash
gedit ~/.bashrc 
export ROS_DOMAIN_ID=22
export ROS_LOCALHOST_ONLY=1 
source ~/.bashrc # apply the changes
```
## INSTALL BUILD DEPENDINCIES

```bash
sudo apt install default-jre #environment for executing Java applications.
```
Micro XRCE-DDS-Gen is required for DDS communication in ROS 2.
```bash
cd
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
```
Opne .bashrc file and add this export line at the and. Save and exit.
```bash
gedit ~/.bashrc 
export PATH=$PATH:~/Micro-XRCE-DDS-Gen/scripts 
source ~/.bashrc # apply the changes
```
After this command you should see a version number but if you see `microxrceddsgen version: null`, please continue with the installation. 

```bash
 microxrceddsgen -version
```
The commands you've provided are for setting up a system to use Git and the GCC ARM toolchain. This may take a few minutes.
```bash
cd
sudo apt-get update 
sudo apt-get install git  
sudo apt-get install gitk git-gui 
sudo apt-get install gcc-arm-none-eabi
```
## CLONE ARDUPILOT
This may take a few minutes.
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```
```bash
git status #check your current status of the git repository
./waf distclean 
./waf distclean 
./waf configure --board MatekF405-Wing 
./waf plane
```
If you run into an error (probably), please try the following steps:
```bash
git submodule init 
git submodule update 
./waf configure --board MatekF405-Wing 
./waf clean 
./waf distclean 
./waf configure --board MatekF405-Wing
```
## ROS 2
```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
```
When you run the following code, you will see a dot at the bottom. It may take some time.
```bash
sudo apt update
sudo apt install python3-vcstool
vcs import --recursive < ros2.repos 
```
Rosdep is a command-line tool used in the Robot Operating System (ROS) to manage package dependencies.
```bash
cd ~/ros2_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```
### TEST ROS 2
This commands to build and test the ardupilot_dds_tests package in your ROS 2 workspace. This may take a few minutes.
```bash
cd ~/ros2_ws
colcon build --packages-up-to ardupilot_dds_tests --cmake-args -DBUILD_TESTING=ON
```
If you'd like to test your installation, run (note: This might take a while. If it takes too long (We think this issue is related to your computer rather than the installation.), don’t wait—just continue with the installation and move on to the next steps:
```bash
source ./install/setup.bash
MAKEFLAGS="-j 1"  colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```
## INSTALL SITL
This may take few minutes.
```bash
cd ~/ardupilot
git pull
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf clean
./waf configure --board sitl
./waf copter -v
```
```bash
cd ~/ardupilot/Tools/autotest
sudo pip3 install MAVProxy
mavproxy.py --version
```
Edit this export line at the end. Save and exit.
```bash
gedit ~/.bashrc
export PATH=$PATH:/path/to/mavproxy
source ~/.bashrc # apply the changes
```
This commands 
```bash
./sim_vehicle.py -v ArduCopter -w

./sim_vehicle.py -v ArduCopter --console --map

./sim_vehicle.py -v ArduCopter -L KSFO --console --map

```
## USE SITL
These are some examples of how to use SITL. You can find more in the document below the video tutorial.
```bash
./sim_vehicle.py -v ArduPlane -f quadplane --console --map --osd 
./sim_vehicle.py -v ArduCopter -f quadcopter --console --map --osd #This runs an ArduPlane simulation in quadplane mode, meaning the aircraft can take off and land vertically like a drone.
```
## ROS2 WITH SITL

```bash
cd ~/ros2_ws/
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
```
If an error occurs, try this one:
```bash
cd ~/ros2_ws/
MAKEFLAGS="-j 1" colcon build --packages-up-to ardupilot_sitl
source install/setup.bash
```
After seeing the connection, you can kill the process with Ctrl + C.
```bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```
## INSTALL GAZEBO GARDEN

```bash
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
```
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```
Install gazebo garden
```bash
sudo apt-get update
sudo apt-get install gz-garden

```
### SITL IN GAZEBO GARDEN
This mat take a few minutes.
```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos
vcs import --recursive < ros2_gz.repos
```
Edit this export line at the end. Save and exit.
```bash
gedit ~/.bashrc
export GZ_VERSION=garden
source ~/.bashrc # apply the changes
```

```bash
cd ~/ros2_ws/
sudo apt update
rosdep update
rosdep install --rosdistro $ROS_DISTRO --from-paths src -i -r -y
```
This may take a few minutes.
```bash
MAKEFLAGS="-j 1" colcon build --packages-up-to ardupilot_gz_bringup --cmake-args -DBUILD_TESTING=ON
```
to test (note: you can see some failures at results. it is normal.)
```bash
source install/setup.bash
colcon test --packages-select ardupilot_sitl ardupilot_dds_tests ardupilot_gazebo ardupilot_gz_applications ardupilot_gz_description ardupilot_gz_gazebo ardupilot_gz_bringup
colcon test-result --all --verbose
```
Add this second line at the end. Save and exit.
```bash
gedit ~/.bashrc
source ~/ros2_ws/install/setup.sh
source ~/.bashrc # apply the changes
```
Finally you can launch one of the example Gazebo simulations.
```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```
```bash
ros2 launch ardupilot_gz_bringup iris_maze.launch.py # a maze example
```

## IMPORTANT NOTES
Please use CTRL+C to close the Gazebo terminal because if there is an active Gazebo simulation running in the background, Gazebo does not work properly.
*If Gazebo opens with a blank screen, try the following steps to resolve the issue:*

```bash
ps aux | grep -E "gz|ros|ardupilot|mavproxy"  # Check running processes
pkill -9 -f "gz|ros|ardupilot|mavproxy"
```



