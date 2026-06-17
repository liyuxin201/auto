cmake_minimum_required(VERSION 3.0.2)
project(mission_auto_avoid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/auto_avoid_manager.py
  scripts/mission_waypoint_uploader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
