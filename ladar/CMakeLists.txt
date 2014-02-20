cmake_minimum_required(VERSION 2.8.3)
project(ladar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	CATKIN_DEPENDS message_runtime
	INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ladarRIData src/ladarRIData.cpp)
add_dependencies(ladarRIData ladar_generate_messages_cpp)
target_link_libraries(ladarRIData ${catkin_LIBRARIES})

add_executable(ladarVisualData src/ladarVisualData.cpp)
add_dependencies(ladarVisualData ladar_generate_messages_cpp)
target_link_libraries(ladarVisualData ${catkin_LIBRARIES})

