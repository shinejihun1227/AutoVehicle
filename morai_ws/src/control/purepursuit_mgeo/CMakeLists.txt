cmake_minimum_required(VERSION 3.0.2)
project(purepursuit_mgeo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  lidar_perception
  visualization_msgs
  morai_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/purepursuit_mgeo_node.py
  scripts/avoidance_frenet_debug_node.py
  scripts/avoidance_path_manager_node.py
  scripts/bypass_lane_guard_node.py
  scripts/highway_lane_strategy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
