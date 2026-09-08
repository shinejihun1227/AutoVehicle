cmake_minimum_required(VERSION 3.0.2)
project(turn_signal_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  morai_msgs
  morai_perception_msgs
  common
)

catkin_python_setup()
catkin_package()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()

catkin_install_python(PROGRAMS
  scripts/turn_signal_node.py
  scripts/maneuver_fusion_node.py
  scripts/inspect_route_signals.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
