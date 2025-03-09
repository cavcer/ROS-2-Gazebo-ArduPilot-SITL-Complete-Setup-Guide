**Video Tutorial at https://www.youtube.com/watch?v=2BhyKyzKAbM&t=1759s**

## ROS 2 Humble Installation

install git to clone repositositories and 
install curl to download necessary files
'''
sudo apt update
sudo apt install git
sudo apt install curl -y
'''
Check and Set UTF-8 Locale
Ensure your system is using UTF-8 encoding to avoid localization issues.
'''
locale  # Check current locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # Verify locale settings
'''
Enable the universe repository to install necessary dependencies.
'''
sudo apt install software-properties-common
sudo add-apt-repository universe
'''
Download and add the ROS 2 repository key to ensure secure package installations.
'''
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
'''
Add the official ROS 2 package repository to your system.
'''
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
'''
'''
sudo apt update
sudo apt upgrade
'''
Add ROS 2 setup to your .bashrc file for automatic sourcing.
'''
gedit ~/.bashrc  # Open .bashrc file
#Add the following line at the end:
source /opt/ros/humble/setup.bash  
source ~/.bashrc  # Apply changes
'''
Set up a workspace for ROS 2 development.
'''
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
'''
Download sample ROS 2 packages for testing. If tehy works, your setup is 
'''
git clone https://github.com/ros/ros_tutorials.git -b humble
'''

Install Dependencies
install missing dependencies using rosdep
'''
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
'''
If rosdep is not found, install it first:
'''
sudo apt install python3-rosdep2
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
'''

Install colcon to build ROS 2 packages.
'''
sudo apt update 
sudo apt install python3-colcon-common-extensions
'''
Compile the workspace using colcon build.
'''
colcon build
'''
Set up the workspace environment before running ROS 2 nodes.
'''
source install/local_setup.bash
'''

Run the turtlesim node to verify that the installation works.
'''
ros2 run turtlesim turtlesim_node
'''
 



