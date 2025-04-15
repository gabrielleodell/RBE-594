cmake_minimum_required(VERSION 3.5)
project(conveyorbelt_msgs)

# ############# ROS2 .msg and .srv ##############
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs roscpp rospy)

add_message_files(DIRECTORY msg FILES ConveyorBeltState.msg)
add_service_files(DIRECTORY srv FILES ConveyorBeltControl.srv)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)
