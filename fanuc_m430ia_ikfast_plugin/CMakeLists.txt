cmake_minimum_required(VERSION 3.0.2)
project(fanuc_m430ia_ikfast_plugin)

find_package(
  catkin REQUIRED COMPONENTS
  tf_conversions
  moveit_core
  pluginlib
  roscpp
)

find_package(LAPACK REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    tf_conversions
    moveit_core
    pluginlib
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(
  include_directories ${PROJECT_NAME}
  libraries ${PROJECT_NAME})

add_library(
  ${PROJECT_NAME}
  src/fanuc_m430ia_ikfast_plugin.cpp
)

install(
  FILES fanuc_m430ia_ikfast_plugin_description.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/fanuc_m430ia_kinematics
)

install(
  TARGETS ${PROJECT_NAME} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${LAPACK_LIBRARIES}
)