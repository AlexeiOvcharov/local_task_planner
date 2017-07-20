cmake_minimum_required(VERSION 2.8.3)
project(local_task_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arm_kinematics
    roscpp
    sensor_msgs
    red_msgs
    std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS arm_kinematics roscpp sensor_msgs std_srvs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(manipulation_control_node
    src/ManipulationControlNode.cpp
    src/Rangefinder.cpp
)

# Node start communication betwenn CV step and Manipulation step 
add_executable(start_node 
	src/main.cpp
	src/ManipulationControlNode.cpp
    src/Rangefinder.cpp
)
target_link_libraries(start_node
    ${catkin_LIBRARIES}
)

# TESTS
add_executable(range_test 
    tests/RangefinderTest.cpp
    src/Rangefinder.cpp
)
target_link_libraries(range_test
    ${catkin_LIBRARIES}
)
