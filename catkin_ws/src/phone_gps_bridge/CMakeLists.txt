cmake_minimum_required(VERSION 3.0.2)
project(phone_gps_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/netgps_tcp_receiver.py
  scripts/gps_udp_receiver.py
  scripts/fake_phone_gps.py
  scripts/gps_track_recorder.py
  scripts/gps_path_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
