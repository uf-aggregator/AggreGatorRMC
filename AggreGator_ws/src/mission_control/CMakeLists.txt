cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_control
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(mission_control_node src/mission_control_node.cpp 
									src/mission_control.cpp 
#									src/state_handler.cpp
									src/mid_level/base_mid_handler.cpp
									src/mid_level/ladar_handler.cpp)
add_dependencies(mission_control_node mission_control_generate_messages_cpp)
target_link_libraries(mission_control_node
   ${catkin_LIBRARIES}
 )