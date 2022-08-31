# Kinova Gen3 Lite MoveIT Examples

This Package covers Robot Manipulation part in Robot Programming Lecture.

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

## Structure of package of gen3_lite_examples
```sh
.
├── CMakeLists.txt
├── package.xml
├── launch
│   ├── gen3_lite_gazebo.launch
│   └── moveit_example.launch
│   └── moveit_example_cpp.launch
│   └── p_n_p.launch
│   └── p_n_p_cpp.launch
│   └── p_n_p2.launch
│   └── p_n_p2_cpp.launch
└── scripts
     └── mave_it
         └── simple_moveit_example.py
         └── simple_moveit_example.cpp
         └── p_n_p.py
         └── p_n_p.cpp
         └── p_n_p2.py
         └── p_n_p2.cpp
```

## Description of Scripts 
* simple_moveit_example
* p_n_p
* p_n_p2
https://github.com/dabarov/moveit-pick-place-python/blob/main/demo.gif
## Usage

1. To start simulation run:
```sh
roslaunch gen3_lite_examples gen3_lite_gazebo.launch
```

2. To start Simple Moving control run:
```sh
roslaunch gen3_lite_examples moveit_example.launch
```
or
```sh
roslaunch gen3_lite_examples moveit_example_cpp.launch
```

3. To start pick and place(tower building) control run:
```sh
roslaunch gen3_lite_examples p_n_p.launch
```
or
```sh
roslaunch gen3_lite_examples p_n_p_cpp.launch
```

4. To start pick and place(moving the bar) control run:
```sh
roslaunch gen3_lite_examples p_n_p2.launch
```
or
```sh
roslaunch gen3_lite_examples p_n_p2_cpp.launch
```
