# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "motorController: 2 messages, 0 services")

set(MSG_I_FLAGS "-ImotorController:/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(motorController_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motorController
)
_generate_msg_cpp(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motorController
)

### Generating Services

### Generating Module File
_generate_module_cpp(motorController
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motorController
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(motorController_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(motorController_generate_messages motorController_generate_messages_cpp)

# target for backward compatibility
add_custom_target(motorController_gencpp)
add_dependencies(motorController_gencpp motorController_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motorController_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motorController
)
_generate_msg_lisp(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motorController
)

### Generating Services

### Generating Module File
_generate_module_lisp(motorController
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motorController
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(motorController_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(motorController_generate_messages motorController_generate_messages_lisp)

# target for backward compatibility
add_custom_target(motorController_genlisp)
add_dependencies(motorController_genlisp motorController_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motorController_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/I2CMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController
)
_generate_msg_py(motorController
  "/home/fnivek/git_hub/NASAboticsCode/AggreGator_ws/src/motorControllerNode/msg/motorMSG.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController
)

### Generating Services

### Generating Module File
_generate_module_py(motorController
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(motorController_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(motorController_generate_messages motorController_generate_messages_py)

# target for backward compatibility
add_custom_target(motorController_genpy)
add_dependencies(motorController_genpy motorController_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS motorController_generate_messages_py)


debug_message(2 "motorController: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motorController)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/motorController
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(motorController_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motorController)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/motorController
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(motorController_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/motorController
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(motorController_generate_messages_py std_msgs_generate_messages_py)
