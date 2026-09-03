cmake_minimum_required(VERSION 3.0.2)
project(morai_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  morai_perception_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/obstacle_fusion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
