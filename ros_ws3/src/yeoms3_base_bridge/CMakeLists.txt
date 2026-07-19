cmake_minimum_required(VERSION 3.0.2)
project(yeoms3_base_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_localizer.py
  scripts/ctrl_cmd_sender.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
