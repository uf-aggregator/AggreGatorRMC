# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motor_controller: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imotor_controller:/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motor_controller_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_controller
)
_generate_msg_cpp(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_controller
)

### Generating Services

### Generating Module File
_generate_module_cpp(motor_controller
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_controller
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motor_controller_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motor_controller_generate_messages motor_controller_generate_messages_cpp)

# target for backward compatibility
add_custom_target(motor_controller_gencpp)
add_dependencies(motor_controller_gencpp motor_controller_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_controller_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_controller
)
_generate_msg_lisp(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_controller
)

### Generating Services

### Generating Module File
_generate_module_lisp(motor_controller
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_controller
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motor_controller_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motor_controller_generate_messages motor_controller_generate_messages_lisp)

# target for backward compatibility
add_custom_target(motor_controller_genlisp)
add_dependencies(motor_controller_genlisp motor_controller_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_controller_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller
)
_generate_msg_py(motor_controller
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller
)

### Generating Services

### Generating Module File
_generate_module_py(motor_controller
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motor_controller_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motor_controller_generate_messages motor_controller_generate_messages_py)

# target for backward compatibility
add_custom_target(motor_controller_genpy)
add_dependencies(motor_controller_genpy motor_controller_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motor_controller_generate_messages_py)


debug_message(2 "motor_controller: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motor_controller
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(motor_controller_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_controller)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motor_controller
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(motor_controller_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motor_controller
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(motor_controller_generate_messages_py std_msgs_generate_messages_py)
