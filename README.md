# Kinova Gen3 Lite MoveIT Examples


### Setup

* OS: Ubuntu 18.04 
* ROS: Melodic


### Build

These are the instructions to run in a terminal to create the workspace, clone the `ros_kortex` repository and install the necessary ROS dependencies:

        sudo apt install python3 python3-pip
        sudo python3 -m pip install conan
        conan config set general.revisions_enabled=1
        conan profile new default --detect > /dev/null
        conan profile update settings.compiler.libcxx=libstdc++11 default
        mkdir -p catkin_workspace/src
        cd catkin_workspace/src
        git clone https://github.com/Kinovarobotics/ros_kortex.git
        git clone https://github.com/AIRLABkhu/gen3_lite_examples.git
        cd ../
        rosdep install --from-paths src --ignore-src -y
        catkin_make
        source devel/setup.bash

