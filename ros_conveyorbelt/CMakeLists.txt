cmake_minimum_required(VERSION 3.5)
project(ros_conveyorbelt)

# ############# COMPILE the ROS2ConveyorBelt Gazebo Plugin ##############

# Find required packages:
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  gazebo_msgs
  gazebo_ros
  roscpp
)

find_package(conveyorbelt_msgs REQUIRED)

# /include:
include_directories(include ${catkin_INCLUDE_DIRS}
  "~/ws_moveit/devel/include")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include
  LIBRARIES
  ros_conveyorbelt

  CATKIN_DEPENDS
  message_runtime
  gazebo_dev
  gazebo_msgs
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)

foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

add_library(ros_conveyorbelt_plugin SHARED
  src/ros_conveyorbelt_plugin.cpp
)
target_link_libraries(ros_conveyorbelt_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# add_dependencies(ros_conveyorbelt_plugin conveyorbelt_msgs_gencpp)
install(TARGETS ros_conveyorbelt_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})