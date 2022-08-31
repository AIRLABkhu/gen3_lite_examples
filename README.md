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
        git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex.git 
        git clone https://github.com/AIRLABkhu/gen3_lite_examples.git
        cd ../
        source /opt/ros/melodic/setup.bash
        rosdep install --from-paths src --ignore-src -y
        catkin_make
        source devel/setup.bash

### make gen3_lite_examples scripts executable
        cd catkin_workspace/src/gen3_lite_examples/src/move_it
        chmod +x p_n_p2.cpp
        chmod +x p_n_p2.py
        chmod +x p_n_p.cpp
        chmod +x p_n_p.py
        chmod +x simple_moveit_example.cpp
        chmod +x simple_moveit_example.py

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

## Troubleshooting
###To run gazebo stably in VMWare, you need to turn-off "Accelerate 3D graphics"
* Open VMWare and Click your workspace.
* Click Edit virtual Machine settings.
* Click Display Option
* Click and turn-off Accelerate 3D graphics



## Description of Scripts 
* simple_moveit_example
1. Planning to a Pose goal
2. Planning by controlling joint state
3. Draw Circle
4. Draw Rectangle
5. Draw A 
* p_n_p
1. Tower Building
* p_n_p2
1. Moving the bar


## Usage

1. To start simulation run:
```sh
roslaunch gen3_lite_examples gen3_lite_gazebo.launch
```
then you will see rviz as below. 
Click File > Open Config
![20220831_123301](https://user-images.githubusercontent.com/75155964/187586935-f7c4f8ce-1ad4-45dc-be82-0c5d9887967d.png)


Click moveit.rviz in move_it_config (move to catkin_workspace > src > gen3_lite_examples > gen3_lite_move_it_config > launch)
![20220831_123341](https://user-images.githubusercontent.com/75155964/187586943-54bc83af-0b2a-4844-bd66-ccb62c9876bd.png)

Click "Close without Saving" then finally you will see the image as below.
![20220831_123943](https://user-images.githubusercontent.com/75155964/187587566-1d1892cf-9a00-42cb-be2b-e6910665e61e.png)


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
