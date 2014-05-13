cmake_minimum_required(VERSION 2.8.3)
project(camera_feedback)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

#generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES camera_feedback
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include ${catkin_INCLUDE_DIRS}
  #${OpenCV_INCLUDE_DIRS}
)

# add_library(camera_feedback
#   src/${PROJECT_NAME}/camera_feedback.cpp
# )

add_executable(camera_feedback_node src/camera_feedback_node.cpp src/camera_feedback.cpp)
add_dependencies(camera_feedback_node camera_feedback_generate_messages_cpp)
#find_package(OpenCV REQUIRED)
target_link_libraries(camera_feedback_node
   ${catkin_LIBRARIES} #${OpenCV_LIBRARIES}
 )
