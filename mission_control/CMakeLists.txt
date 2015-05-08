cmake_minimum_required(VERSION 2.8.3)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common_files
  genmsg
  hardware_interface
)

################################################
## Declare ROS messages, services and actions ##
################################################


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_control
  CATKIN_DEPENDS roscpp rospy std_msgs common_files
)

###########
## Build ##
###########

#create library for linking definitions
add_library(
	mission_utils
  include/mission_control/behaviors/orientation_behavior.cpp
  include/mission_control/behaviors/motor_utility.cpp
  include/mission_control/stateMachines/state_start.cpp
  include/mission_control/stateMachines/state_navigation.cpp
  include/mission_control/stateMachines/state_orientation.cpp
  include/mission_control/stateMachines/state_mine.cpp
  include/mission_control/stateMachines/state_dump.cpp
  include/mission_control/stateMachines/state_machine.cpp
	include/mission_control/mission_control.cpp
)

#make the include directory known
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
