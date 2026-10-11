cmake_minimum_required(VERSION 3.0.2)
project(tron_local_trajectory_planner)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/local_trajectory_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
