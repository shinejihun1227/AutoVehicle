cmake_minimum_required(VERSION 3.0.2)
project(stopline_control)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  morai_perception_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(PROGRAMS
  scripts/stopline_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
