cmake_minimum_required(VERSION 3.0.2)
project(morai_camera_perception)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  morai_perception_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/camera_health_monitor.py
  scripts/camera_lane_detector.py
  scripts/traffic_light_detector.py
  scripts/camera_obstacle_detector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
