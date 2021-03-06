cmake_minimum_required(VERSION 2.8.3)
project(common_files)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


add_message_files (DIRECTORY msg
  FILES
  Drive.msg
  Encoders.msg 
  Bucket.msg
  Ladder.msg
  Centroid.msg
  IRDistance.msg
  IRDistances.msg
  WriteI2C.msg
  WriteI2CRegister.msg
  RawIRData.msg
  RawMotorPowerData.msg
  ElectronicPowerData.msg
  keys.msg
  Avg_Noise.msg
  Filtered.msg
  Gyro.msg
#  CurrentSense.msg
)

add_service_files (DIRECTORY srv 
	FILES
	ReadI2C.srv
	ReadI2CRegister.srv
	ReadLidar.srv
	Coordinates.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
) 

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES common_files
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(common_files_node src/common_files_node.cpp)
add_dependencies(common_files_node common_files_generate_messages_cpp)
target_link_libraries(common_files_node ${catkin_LIBRARIES})

