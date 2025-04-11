cmake_minimum_required(VERSION 3.1.3)
project(fanuc_kinematics_plugin)

find_package(
  catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_kinematics
  geometry_msgs
  moveit_core
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  
    moveit_ros_planning
    moveit_kinematics
    geometry_msgs
    moveit_core
    pluginlib
    roscpp
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(
  include_directories ${PROJECT_NAME}
  libraries ${PROJECT_NAME}
)

add_library(
  ${PROJECT_NAME} 
  src/fanuc_kinematics_plugin.cpp
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)