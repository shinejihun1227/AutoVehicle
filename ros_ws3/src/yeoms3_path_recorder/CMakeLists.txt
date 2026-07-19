cmake_minimum_required(VERSION 3.0.2)
project(yeoms3_path_recorder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/path_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
