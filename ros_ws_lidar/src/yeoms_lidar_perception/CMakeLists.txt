cmake_minimum_required(VERSION 3.0.2)
project(yeoms_lidar_perception)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/lidar_obstacle_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
