cmake_minimum_required(VERSION 3.0.2)
project(tron_nav_dashboard)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  tron_local_collision_safety
  tron_open_space_nav
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  tron_local_collision_safety
  tron_open_space_nav
)

catkin_install_python(PROGRAMS
  scripts/navigation_debug_monitor.py
  scripts/navigation_dashboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
