cmake_minimum_required(VERSION 3.0.2)
project(hri_engagement)


find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  hri_msgs
  geometry_msgs
  tf
)

catkin_install_python(PROGRAMS script/engagement_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

