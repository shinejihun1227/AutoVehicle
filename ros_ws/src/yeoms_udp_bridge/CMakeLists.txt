cmake_minimum_required(VERSION 3.0.2)
project(yeoms_udp_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/udp_packet_monitor.py
  scripts/udp_status_to_localization.py
  scripts/udp_ctrl_cmd_sender.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

