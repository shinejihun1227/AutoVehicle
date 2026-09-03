cmake_minimum_required(VERSION 3.0.2)
project(morai_perception_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  Obstacle.msg
  ObstacleArray.msg
  SafetyStop.msg
  LaneDetection.msg
  TrafficLight.msg
  GpsHealth.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
