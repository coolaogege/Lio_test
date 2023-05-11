# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lili_om_rot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ilili_om_rot:/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lili_om_rot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_custom_target(_lili_om_rot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lili_om_rot" "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lili_om_rot
  "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lili_om_rot
)

### Generating Services

### Generating Module File
_generate_module_cpp(lili_om_rot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lili_om_rot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lili_om_rot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lili_om_rot_generate_messages lili_om_rot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_dependencies(lili_om_rot_generate_messages_cpp _lili_om_rot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lili_om_rot_gencpp)
add_dependencies(lili_om_rot_gencpp lili_om_rot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lili_om_rot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lili_om_rot
  "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lili_om_rot
)

### Generating Services

### Generating Module File
_generate_module_eus(lili_om_rot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lili_om_rot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lili_om_rot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lili_om_rot_generate_messages lili_om_rot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_dependencies(lili_om_rot_generate_messages_eus _lili_om_rot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lili_om_rot_geneus)
add_dependencies(lili_om_rot_geneus lili_om_rot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lili_om_rot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lili_om_rot
  "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lili_om_rot
)

### Generating Services

### Generating Module File
_generate_module_lisp(lili_om_rot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lili_om_rot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lili_om_rot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lili_om_rot_generate_messages lili_om_rot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_dependencies(lili_om_rot_generate_messages_lisp _lili_om_rot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lili_om_rot_genlisp)
add_dependencies(lili_om_rot_genlisp lili_om_rot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lili_om_rot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lili_om_rot
  "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lili_om_rot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lili_om_rot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lili_om_rot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lili_om_rot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lili_om_rot_generate_messages lili_om_rot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_dependencies(lili_om_rot_generate_messages_nodejs _lili_om_rot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lili_om_rot_gennodejs)
add_dependencies(lili_om_rot_gennodejs lili_om_rot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lili_om_rot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lili_om_rot
  "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lili_om_rot
)

### Generating Services

### Generating Module File
_generate_module_py(lili_om_rot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lili_om_rot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lili_om_rot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lili_om_rot_generate_messages lili_om_rot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lili-om-modify_ws/src/LiLi-OM-ROT/msg/cloud_info.msg" NAME_WE)
add_dependencies(lili_om_rot_generate_messages_py _lili_om_rot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lili_om_rot_genpy)
add_dependencies(lili_om_rot_genpy lili_om_rot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lili_om_rot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lili_om_rot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lili_om_rot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lili_om_rot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lili_om_rot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lili_om_rot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lili_om_rot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lili_om_rot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lili_om_rot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lili_om_rot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lili_om_rot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lili_om_rot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lili_om_rot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lili_om_rot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lili_om_rot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lili_om_rot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lili_om_rot_generate_messages_py std_msgs_generate_messages_py)
endif()
