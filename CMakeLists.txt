cmake_minimum_required(VERSION 3.0.2)
project(engagement_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hri_msgs
  rospy
  std_msgs
)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
   script/demo.py 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

