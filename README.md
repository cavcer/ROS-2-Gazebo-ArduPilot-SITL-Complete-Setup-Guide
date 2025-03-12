# ROS 2 Gazebo ArduPilot SITL Complete Setup Guide

## Video Tutorial

A step-by-step video tutorial is available at: [YouTube Video](https://www.youtube.com/watch?v=2BhyKyzKAbM\&t=1759s)

## Install Git and Curl

Ensure Git is installed for cloning repositories and Curl for downloading necessary files.

```bash
sudo apt update
sudo apt install git curl -y
```

## Check and Set UTF-8 Locale

Ensure your system uses UTF-8 encoding to avoid localization issues.

```bash
locale  # Check current locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verify locale settings
```

## Enable Universe Repository

Enable the Universe repository to install necessary dependencies.

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe  # Press Enter
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

### Setup Environment

Automatically source ROS 2 setup when opening a terminal.

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc  # Apply the changes
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

Install Colcon to build ROS 2 packages.

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

### Build and Source Workspace

Compile the workspace using Colcon.

```bash
colcon build
```

Set up the workspace environment before running ROS 2 nodes.

```bash
source install/local_setup.bash
```

### Test Installation

Run the Turtlesim node to verify the installation.

```bash
ros2 run turtlesim turtlesim_node
```

If Turtlesim launches successfully, your ROS 2 installation is complete!

## Source ROS 2 Environment and Check

Verify that variables like `ROS_DISTRO` and `ROS_VERSION` are set.

```bash
printenv | grep -i ROS
```

Run this code, it will edit your .bashrc file.

```bash
echo 'export ROS_DOMAIN_ID=22' >> ~/.bashrc
echo 'export ROS_LOCALHOST_ONLY=1' >> ~/.bashrc
source ~/.bashrc  # Apply the changes
```

## Install Build Dependencies

```bash
sudo apt install default-jre  # Java runtime environment for executing applications
```

Micro XRCE-DDS-Gen is required for DDS communication in ROS 2.

```bash
cd
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble
```

This code will edit your .bashrc file:

```bash
echo 'export PATH=$PATH:~/Micro-XRCE-DDS-Gen/scripts' >> ~/.bashrc
source ~/.bashrc  # Apply the changes
```

Check if Micro XRCE-DDS-Gen is installed correctly (note: you should see a version number but if you see version: null, please continue to installation.):

```bash
microxrceddsgen -version
```

## Clone ArduPilot

This may take a few minutes.

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

Check repository status and clean the build environment:

```bash
git status  # Check current Git repository status
./waf distclean
./waf configure --board MatekF405-Wing 
./waf plane
```

If errors occur, try:

```bash
git submodule init 
git submodule update 
./waf configure --board MatekF405-Wing 
./waf clean 
./waf distclean 
./waf configure --board MatekF405-Wing
```

## Install Gazebo Garden

```bash
sudo apt-get update
sudo apt-get install lsb-release curl gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
```

Add the repository and install Gazebo Garden.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

## SITL in Gazebo Garden

This may take a few minutes.

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos
vcs import --recursive < ros2_gz.repos
```

Edit the `.bashrc` file and add:

```bash
echo 'export GZ_VERSION=garden' >> ~/.bashrc
source ~/.bashrc
```

Build the package:

```bash
cd ~/ros2_ws/
sudo apt update
rosdep update
rosdep install --rosdistro $ROS_DISTRO --from-paths src -i -r -y
colcon build --packages-up-to ardupilot_gz_bringup --cmake-args -DBUILD_TESTING=ON
```

## Launch Gazebo Simulations

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
ros2 launch ardupilot_gz_bringup iris_maze.launch.py  # A maze example
```

### Important Notes

Use `CTRL+C` to close Gazebo properly. If Gazebo opens with a blank screen:

```bash
ps aux | grep -E "gz|ros|ardupilot|mavproxy"  # Check running processes
pkill -9 -f "gz|ros|ardupilot|mavproxy"
```

