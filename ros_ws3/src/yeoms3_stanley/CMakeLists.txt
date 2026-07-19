cmake_minimum_required(VERSION 3.0.2)
project(yeoms3_stanley)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/stanley_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
