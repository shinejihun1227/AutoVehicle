cmake_minimum_required(VERSION 3.0.2)
project(yeoms_pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pure_pursuit_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
