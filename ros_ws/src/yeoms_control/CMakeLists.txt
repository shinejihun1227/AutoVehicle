cmake_minimum_required(VERSION 3.0.2)
project(yeoms_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/stanley_controller.py
  scripts/waypoint_recorder.py
  scripts/waypoint_origin.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch waypoints
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
