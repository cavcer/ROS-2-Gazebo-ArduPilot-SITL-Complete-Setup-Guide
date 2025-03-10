# ROS 2 Humble Installation Guide

## 1. Set Locale
```bash
locale # Check for UTF-8 support

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale # Verify settings
```

## 2. Add Required Repositories
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## 3. Install ROS 2 Humble
```bash
sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
```

## 4. Setup Environment
```bash
gedit ~/.bashrc
source /opt/ros/humble/setup.bash # Add this line to the file
source ~/.bashrc
```

## 5. Create a Workspace and Clone Example Repository
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Install Git if not available
sudo apt install git

git clone https://github.com/ros/ros_tutorials.git -b humble
```

## 6. Install Dependencies and Build
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y

# If 'rosdep' command is not found:
sudo apt install python3-rosdep2
rosdep update
rosdep install -i --from-path src --rosdistro humble -y

sudo apt update
sudo apt install python3-colcon-common-extensions

colcon build
source install/local_setup.bash
```

## 7. Test Installation
```bash
ros2 run turtlesim turtlesim_node # If this works, the installation is successful
```

## 8. Verify ROS 2 Environment Variables
```bash
printenv | grep -i ROS # Ensure ROS_DISTRO and ROS_VERSION are set

gedit ~/.bashrc
export ROS_DOMAIN_ID=22
export ROS_LOCALHOST_ONLY=1
source ~/.bashrc
```

## 9. Install Build Dependencies
```bash
sudo apt install default-jre

cd ~ # Exit ros2_ws

git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd Micro-XRCE-DDS-Gen
./gradlew assemble

gedit ~/.bashrc
export PATH=$PATH:~/Micro-XRCE-DDS-Gen/scripts
source ~/.bashrc

microxrceddsgen -version # Check installation
```

## 10. Install ArduPilot and Compile Firmware
```bash
cd ~
sudo apt update
sudo apt install git gitk git-gui gcc-arm-none-eabi

git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot

git status
./waf distclean
./waf configure --board MatekF405-Wing
./waf plane
```

## 11. Install ROS 2 for ArduPilot
```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos

sudo apt update
sudo apt install python3-vcstool
vcs import --recursive < ros2.repos

cd ~/ros2_ws
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-up-to ardupilot_dds_tests --cmake-args -DBUILD_TESTING=ON
```

## 12. Test ArduPilot DDS
```bash
cd ~/ros2_ws
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```

## 13. Install and Use SITL
```bash
cd ~/ardupilot
git pull
Tools/environment_install/install-prereqs-ubuntu.sh -y

./waf clean
./waf configure --board sitl
./waf copter -v

cd ~/ardupilot/Tools/autotest

sudo pip3 install MAVProxy
mavproxy.py --version

gedit ~/.bashrc
export PATH=$PATH:/path/to/mavproxy
source ~/.bashrc

./sim_vehicle.py -v ArduCopter --console --map
```

## 14. Run SITL with ROS 2
```bash
cd ~/ros2_ws/
colcon build --packages-up-to ardupilot_sitl
source install/setup.bash

ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```

## 15. Install Gazebo Garden
```bash
sudo apt update
sudo apt install lsb-release curl gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt update
sudo apt install gz-garden
```

## 16. Use SITL in Gazebo Garden
```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos
vcs import --recursive < ros2_gz.repos

gedit ~/.bashrc
export GZ_VERSION=garden
source ~/.bashrc

cd ~/ros2_ws
rosdep update
rosdep install --rosdistro $ROS_DISTRO --from-paths src -i -r -y

colcon build --packages-up-to ardupilot_gz_bringup --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
```

## 17. Verify ROS 2 with SITL and Gazebo
```bash
ros2 topic list
ros2 topic echo /imu
```

