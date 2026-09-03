cmake_minimum_required(VERSION 3.0.2)
project(lidar_obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  morai_perception_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lidar_obstacle_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
