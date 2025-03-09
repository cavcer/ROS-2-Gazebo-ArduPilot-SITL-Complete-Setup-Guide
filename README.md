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

### Add ROS 2 to .bashrc
Automatically source ROS 2 setup when opening a terminal.

```bash
gedit ~/.bashrc  # Open .bashrc file
```

Add the following line at the end , save and exit:

```bash
source /opt/ros/humble/setup.bash
```

Apply changes:

```bash
source ~/.bashrc
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

