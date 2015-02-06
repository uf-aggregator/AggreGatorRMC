cmake_minimum_required(VERSION 2.8.3)
project(ir_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  hardware_interface
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ir_processing hardware_utils
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

add_library(
	ir_processing_utils
	include/${PROJECT_NAME}/ir_processing.cpp
)

add_executable(
	ir_processing_node 
	src/ir_processing_node.cpp
)

add_dependencies(
	ir_processing_node ir_processing_generate_messages_cpp
)

target_link_libraries(
	ir_processing_node
	${catkin_LIBRARIES}
	ir_processing_utils
)
