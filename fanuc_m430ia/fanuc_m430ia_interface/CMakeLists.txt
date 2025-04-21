cmake_minimum_required(VERSION 3.1.3)
project(fanuc_m430ia_interface)

find_package(
  catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_planning
  visualization_msgs
  conveyorbelt_msgs
  geometry_msgs
  moveit_core
  rospy
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  moveit_ros_planning_interface
  moveit_ros_planning
  visualization_msgs
  geometry_msgs
  moveit_core
  rospy
  DEPENDS system_lib
)

catkin_install_python(
  PROGRAMS
  src/interface.py
  src/chutes.py
  src/estop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)