cmake_minimum_required(VERSION 3.0.2)
project(tron_sensor_bridge)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/check_sensor_chain.sh
  scripts/start_sensor.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
