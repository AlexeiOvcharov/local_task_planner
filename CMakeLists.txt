cmake_minimum_required(VERSION 2.8.3)
project(local_task_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    arm_kinematics
    roscpp
    tf2_ros
    sensor_msgs
    red_msgs
    std_srvs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS arm_kinematics roscpp sensor_msgs std_srvs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(FirstTest src/main.cpp src/LTP.cpp)

target_link_libraries(FirstTest ${catkin_LIBRARIES})