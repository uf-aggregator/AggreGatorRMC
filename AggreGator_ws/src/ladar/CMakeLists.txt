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
	
	#INCLUDE_DIRS include				#Not working for some reason
	CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(ladar_map_node src/ladarPrintMap.cpp src/v1/interpretLadar.cpp)
#add_dependencies(ladar_map_node ladar_generate_messages_cpp)
#target_link_libraries(ladar_map_node ${catkin_LIBRARIES})

add_executable(ladar_print_node src/v1/ladarPrintRange.cpp src/v1/interpretLadar.cpp)
add_dependencies(ladar_print_node ladar_generate_messages_cpp)
target_link_libraries(ladar_print_node ${catkin_LIBRARIES})

add_executable(ladar_range_node src/v1/ladarRange.cpp src/v1/interpretLadar.cpp)
add_dependencies(ladar_range_node ladar_generate_messages_cpp)
target_link_libraries(ladar_range_node ${catkin_LIBRARIES})

#add_executable(tf_pub_node src/tfPub.cpp)
#add_executable(tf_sub_node src/tfSub.cpp)
#target_link_libraries(tf_pub_node ${catkin_LIBRARIES})
#target_link_libraries(tf_sub_node ${catkin_LIBRARIES})