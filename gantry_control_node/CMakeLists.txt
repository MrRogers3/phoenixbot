cmake_minimum_required(VERSION 2.8.3)
project(gantry_control_node)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   geometry_msgs
   trajectory_msgs
   genmsg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(project_sources scripts/gantry_control_node.cpp)
add_executable(gantry_control_node ${project_sources})
target_link_libraries(gantry_control_node ${catkin_LIBRARIES})
