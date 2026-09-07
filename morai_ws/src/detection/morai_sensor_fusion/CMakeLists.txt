cmake_minimum_required(VERSION 3.0.2)
project(morai_sensor_fusion)

find_package(catkin REQUIRED COMPONENTS
  lidar_perception
  morai_perception_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS lidar_perception morai_perception_msgs nav_msgs rospy std_msgs)

catkin_install_python(PROGRAMS
  scripts/obstacle_fusion_node.py
  scripts/roi_sensor_safety_adapter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
