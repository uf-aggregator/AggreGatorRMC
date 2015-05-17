cmake_minimum_required(VERSION 2.8.3)
project(remote_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  genmsg
  std_msgs
  sensor_msgs
  common_files
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs common_files
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

#################
## Executables ##
#################
add_executable(xbox_controller_node src/xbox_controller_node.cpp)
add_dependencies(xbox_controller_node xbox_controller_generate_messages_cpp)
target_link_libraries(xbox_controller_node ${catkin_LIBRARIES})



