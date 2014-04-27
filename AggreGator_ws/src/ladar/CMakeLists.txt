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
	
	INCLUDE_DIRS include				#Not working for some reason
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ladar_print_node src/v1/ladar_print_node.cpp)
add_dependencies(ladar_print_node ladar_generate_messages_cpp)
target_link_libraries(ladar_print_node ${catkin_LIBRARIES})

#add_executable(ladar_node src/v1/ladar_node.cpp src/v1/interpret_ladar.cpp src/v1/draw.cpp src/v1/localization.cpp)
#add_dependencies(ladar_node ladar_generate_messages_cpp)
#target_link_libraries(ladar_node ${catkin_LIBRARIES} SDL)