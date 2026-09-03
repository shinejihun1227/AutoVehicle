cmake_minimum_required(VERSION 3.0.2)
project(sensor_noise_estimator)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  NoiseStatistics.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS message_runtime nav_msgs rospy sensor_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/sensor_noise_estimator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
