cmake_minimum_required(VERSION 3.0.2)
project(tron_open_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_localizer.py
  scripts/gps_goal_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
