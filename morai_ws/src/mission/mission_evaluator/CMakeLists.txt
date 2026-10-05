cmake_minimum_required(VERSION 3.0.2)
project(mission_evaluator)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  morai_perception_msgs
  nav_msgs
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mission_evaluator_node.py
  scripts/competition_lap_node.py
  scripts/collision_evaluation_bridge.py
  scripts/inspect_competition_course.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
