#!/bin/sh

sudo apt install python3 python3-pip
sudo python3 -m pip install conan
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
mkdir -p catkin_ws/src
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex.git 
git clone https://github.com/AIRLABkhu/gen3_lite_examples.git
cd ../
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
rosdep install --from-paths src --ignore-src -y
catkin_make
echo "source devel/setup.bash" >> ~/.bashrc
source /opt/ros/melodic/setup.bash
source devel/setup.bash
