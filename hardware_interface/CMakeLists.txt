cmake_minimum_required(VERSION 2.8.3)
project(hardware_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  common_files
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
  INCLUDE_DIRS include
  LIBRARIES hardware_utils
  CATKIN_DEPENDS roscpp rospy std_msgs common_files
# DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
 add_library(
    hardware_utils
      include/hardware_interface/GPIO.cpp
      include/hardware_interface/i2c.cpp
      include/hardware_interface/ina226.cpp
 )

## Declare adc cpp executable
add_executable(adc_node src/adc_node.cpp)
target_link_libraries(adc_node ${catkin_LIBRARIES} hardware_utils)

## Declare ina226 cpp executable
add_executable(ina226_node src/ina226_node.cpp)
target_link_libraries(ina226_node ${catkin_LIBRARIES} hardware_utils)

## Declare a i2c cpp executable
add_executable(i2c_node src/i2c_node.cpp)
target_link_libraries(i2c_node ${catkin_LIBRARIES} hardware_utils)

# Declare test node
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} hardware_utils)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(adc_node ina226_node i2c_node test_node hardware_interface_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS hardware_utils 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )


