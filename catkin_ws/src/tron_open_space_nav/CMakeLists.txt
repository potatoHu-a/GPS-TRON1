cmake_minimum_required(VERSION 3.0.2)
project(tron_open_space_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  MissionStatus.msg
  LocalizationStatus.msg
  ObstacleStatus.msg
  WaypointInfo.msg
  WaypointInfoArray.msg
)

add_service_files(
  FILES
  MissionPatrolStart.srv
  ResetLocalizationDistance.srv
  SetMeasuredDistance.srv
  DeleteWaypoint.srv
  MoveWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs visualization_msgs tf2_ros
  INCLUDE_DIRS include
  LIBRARIES tron_open_space_nav_mapviz_plugins
)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(Qt5 REQUIRED COMPONENTS Core Widgets OpenGL)

set(CMAKE_AUTOMOC ON)
add_definitions(-DQT_NO_KEYWORDS)

find_package(mapviz QUIET)
find_package(pluginlib QUIET)

if(mapviz_FOUND AND pluginlib_FOUND)
  add_library(tron_open_space_nav_mapviz_plugins
    src/open_nav_mission_panel.cpp
    include/tron_open_space_nav/open_nav_mission_panel.h
  )
  target_link_libraries(tron_open_space_nav_mapviz_plugins
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::OpenGL
  )
  add_dependencies(tron_open_space_nav_mapviz_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  get_filename_component(
    WORKSPACE_DEVEL_LIB
    "${CATKIN_DEVEL_PREFIX}/../../../devel_isolated/${PROJECT_NAME}/lib"
    ABSOLUTE
  )
  add_custom_command(TARGET tron_open_space_nav_mapviz_plugins POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CATKIN_DEVEL_PREFIX}/lib
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:tron_open_space_nav_mapviz_plugins>
      ${CATKIN_DEVEL_PREFIX}/lib/
    COMMAND ${CMAKE_COMMAND} -E make_directory ${WORKSPACE_DEVEL_LIB}
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:tron_open_space_nav_mapviz_plugins>
      ${WORKSPACE_DEVEL_LIB}/
  )

  install(TARGETS tron_open_space_nav_mapviz_plugins
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
  install(DIRECTORY plugin/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/plugin
  )
  message(STATUS "tron_open_space_nav: building OpenNavMissionPanel Mapviz plugin")
else()
  message(WARNING "tron_open_space_nav: mapviz/pluginlib not found — skipping Mapviz plugin build")
endif()

catkin_install_python(PROGRAMS
  scripts/geodetic_utils.py
  scripts/gps_enu_converter.py
  scripts/gps_global_converter.py
  scripts/gps_waypoint_collector.py
  scripts/gps_multi_waypoint_collector.py
  scripts/mapviz_tf_broadcaster.py
  scripts/open_nav_path_publisher.py
  scripts/tile_http_server.py
  scripts/start_mapviz_when_ready.py
  scripts/mapviz_click_to_goal.py
  scripts/patrol_mission_manager.py
  scripts/localization_diagnostics.py
  scripts/test_geodetic_utils.py
  scripts/gps_fastlio_fusion.py
  scripts/waypoint_tracker.py
  scripts/lidar_obstacle_avoid.py
  scripts/tron_controller_bridge.py
  scripts/nav_mode_manager.py
  scripts/fake_fastlio_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/check_system.sh
  scripts/start_gps_nav.sh
  scripts/start_mapviz.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config docs include maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
