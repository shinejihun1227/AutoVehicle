cmake_minimum_required(VERSION 3.0.2)
project(yeoms_lidar_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/vlp16_udp_receiver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
