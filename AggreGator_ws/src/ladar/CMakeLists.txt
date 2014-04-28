cmake_minimum_required(VERSION 2.8.3)
project(ladar)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
 
add_message_files (FILES processed_data.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
	INCLUDE_DIRS include	#Not working on oDroid
	CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
    )

add_executable(ladar_print_node src/ladar_print_node.cpp)
add_dependencies(ladar_print_node ladar_generate_messages_cpp)
target_link_libraries(ladar_print_node ${catkin_LIBRARIES})

add_executable(ladar_node src/ladar_node.cpp src/localization.cpp src/draw.cpp src/interpret_ladar.cpp src/conversions.cpp
)
add_dependencies(ladar_node ladar_generate_messages_cpp)
target_link_libraries(ladar_node ${catkin_LIBRARIES} SDL)

add_executable(test_node src/test_publishers.cpp)
add_dependencies(test_node ladar_generate_messages_cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

