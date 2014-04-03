cmake_minimum_required(VERSION 2.8.3)
project(i2c)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  hardware_interface
)

 add_message_files(
   FILES
   I2CMSG.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime hardware_interface
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(i2c
#   src/${PROJECT_NAME}/i2c.cpp
# )

 add_executable(i2c_node src/i2c_node.cpp)
 add_dependencies(i2c_node i2c_generate_messages_cpp)
 target_link_libraries(i2c_node ${catkin_LIBRARIES})