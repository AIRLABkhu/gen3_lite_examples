
cmake_minimum_required(VERSION 2.8.3)
project(gen3_lite_examples)

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation actionlib_msgs actionlib kortex_driver moveit_ros_planning_interface moveit_visual_tools)
find_package(Eigen3 REQUIRED)
## Declare a catkin package
catkin_package()

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${PROJECT_SOURCE_DIR}/src)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/client_stubs)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/messages)
include_directories(include ${PROJECT_SOURCE_DIR}/../kortex_api/include/common)

add_executable(simple_moveit_example_cpp src/move_it/simple_moveit_example.cpp)
add_dependencies(simple_moveit_example_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_moveit_example_cpp ${catkin_LIBRARIES} )

add_executable(p_n_p_cpp src/move_it/p_n_p.cpp)
add_dependencies(p_n_p_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(p_n_p_cpp ${catkin_LIBRARIES} )

add_executable(p_n_p2_cpp src/move_it/p_n_p2.cpp)
add_dependencies(p_n_p2_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(p_n_p2_cpp ${catkin_LIBRARIES} )


# Install

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS 
			       src/move_it/p_n_p.py
			       src/move_it/p_n_p2.py
			       src/move_it/simple_moveit_example.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables and/or libraries for installation
install(TARGETS 
    simple_moveit_example_cpp
    p_n_p_cpp
    p_n_p2_cpp
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
