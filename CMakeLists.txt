cmake_minimum_required(VERSION 3.0.2)
project(hri_engagement)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  tf
)

catkin_install_python(PROGRAMS script/engagement_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
