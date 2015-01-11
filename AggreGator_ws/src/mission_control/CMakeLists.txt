cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  hardware_interface
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

 generate_messages(
   DEPENDENCIES
   std_msgs
   hardware_interface
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_control hardware_utils
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

add_library(
	mission_utils
	include/mission_control/behaviors/behaviors.cpp
  include/mission_control/behaviors/behavior_move.cpp
	include/mission_control/stateMachine/state_machine.cpp
	include/mission_control/mission_control.cpp
)

include_directories(
	${catkin_INCLUDE_DIRS}
  	include ${catkin_INCLUDE_DIRS}
)

add_executable(mission_control_node src/mission_control_node.cpp )

add_dependencies(mission_control_node mission_control_generate_messages_cpp)

target_link_libraries(mission_control_node
   ${catkin_LIBRARIES}
   mission_utils
 )