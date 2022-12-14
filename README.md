# Kinova Gen3 Lite with MoveIT 

This Package covers Robot Manipulation part in Robot Programming Lecture.

Note that: you must use C++ in lecture and assignment (Python is not permitted)

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
        mkdir -p catkin_ws/src
        cd catkin_ws/src
        git clone -b melodic-devel https://github.com/Kinovarobotics/ros_kortex.git 
        git clone https://github.com/AIRLABkhu/gen3_lite_examples.git
        cd ../
        source /opt/ros/melodic/setup.bash
        rosdep install --from-paths src --ignore-src -y
        catkin_make
        source devel/setup.bash

### make gen3_lite_examples scripts executable
        cd catkin_ws/src/gen3_lite_examples/src/move_it
        chmod +x khu_moveit_sample.cpp


## Structure of package of gen3_lite_examples
```sh
.
├── CMakeLists.txt
├── package.xml
├── ros_kortex_setup.sh
├── launch
│   ├── gen3_khu_gazebo.launch
│   └── gen3_khu_moveit.launch
└── scripts
     └── move_it
         └── khu_moveit_sample.cpp
```

## Troubleshooting
### To run gazebo stably in VMWare, you need to turn-off "Accelerate 3D graphics"
* Open VMWare and Click your workspace.
* Click Edit virtual Machine settings.
* Click Display Option
* Click and turn-off Accelerate 3D graphics

## Usage

1. To launch gazebo:
```sh
roslaunch gen3_lite_examples gen3_khu_gazebo.launch
```

Note that: If you make your own script, then your should add your script in CMakeLists.txt and when you modify code, you should catkin_make build again.

2. To launch script :
```sh
roslaunch gen3_lite_examples gen3_khu_moveit.launch
```


