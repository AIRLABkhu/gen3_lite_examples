#!/bin/bash

echo ""
echo "[Install MoveIt]"
echo ""
sudo apt install ros-melodic-moveit ros-melodic-moveit-visual-tools ros-melodic-joint-* ros-melodic-ros-control ros-melodic-ros-controllers -y

echo ""
echo "[Build]"
echo ""
sudo apt install python3 python3-pip
sudo python3 -m pip install conan==1.59
conan config set general.revisions_enabled=1
conan profile new default --detect > /dev/null
conan profile update settings.compiler.libcxx=libstdc++11 default
mkdir -p catkin_ws/src
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex.git 
git clone https://github.com/AIRLABkhu/gen3_lite_examples.git
cd ../
rosdep install --from-paths src --ignore-src -y
catkin_make

source $HOME/.bashrc

echo ""
echo "[Complete!]"
