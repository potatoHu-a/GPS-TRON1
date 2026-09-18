cmake_minimum_required(VERSION 3.0.2)
project(tron_local_collision_safety)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

add_message_files(
  FILES
  ObstacleStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  rosgraph
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/collision_safety.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_runtime.sh
  scripts/stop_legacy_obstacle.sh
  scripts/start_collision_safety.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config docs launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
