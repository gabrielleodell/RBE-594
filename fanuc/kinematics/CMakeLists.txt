cmake_minimum_required(VERSION 3.0.2)
project(kinematics)

find_package(catkin REQUIRED COMPONENTS
  moveit_kinematics
  moveit_core
  pluginlib
  std_msgs
  roscpp
  rospy
)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  FK.msg
)

add_service_files(
  FILES
  InverseKinematics.srv
  ForwardVelocity.srv
  InverseVelocity.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS  
  message_generation 
  moveit_kinematics
  message_runtime
  moveit_core
  pluginlib
  std_msgs
  roscpp
  rospy 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
  src/inverse_kinematics.py
  src/forward_kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)