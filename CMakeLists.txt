cmake_minimum_required(VERSION 3.0.2)
project(hri_engagement)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  tf
  pyhri
)

catkin_python_setup()

catkin_install_python(PROGRAMS script/engagement_monitor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_hri_engagement.test)
  add_rostest(test/test_hri_engagement_synth_data.test)
endif()



