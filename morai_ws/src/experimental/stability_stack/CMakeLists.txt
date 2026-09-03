cmake_minimum_required(VERSION 3.0.2)
project(stability_stack)

find_package(catkin REQUIRED COMPONENTS
  morai_msgs
  morai_perception_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_noise_injector.py
  scripts/imu_noise_injector.py
  scripts/imu_robust_filter.py
  scripts/gps_robust_filter.py
  scripts/camera_stability_controller.py
  scripts/camera_localization_fallback_controller.py
  scripts/sensor_quality_monitor.py
  scripts/gps_blackout_detector.py
  scripts/gps_blackout_stability_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
