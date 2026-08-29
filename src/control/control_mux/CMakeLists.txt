cmake_minimum_required(VERSION 3.0.2)
project(control_mux)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  morai_perception_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/control_mux_node.py
  scripts/control_mux_test_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
