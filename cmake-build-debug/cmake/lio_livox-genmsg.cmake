# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lio_livox: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ilio_livox:/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lio_livox_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_custom_target(_lio_livox_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lio_livox" "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" ""
)

get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_custom_target(_lio_livox_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lio_livox" "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lio_livox
)

### Generating Services
_generate_srv_cpp(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lio_livox
)

### Generating Module File
_generate_module_cpp(lio_livox
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lio_livox
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lio_livox_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lio_livox_generate_messages lio_livox_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_dependencies(lio_livox_generate_messages_cpp _lio_livox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_dependencies(lio_livox_generate_messages_cpp _lio_livox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lio_livox_gencpp)
add_dependencies(lio_livox_gencpp lio_livox_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lio_livox_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lio_livox
)

### Generating Services
_generate_srv_eus(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lio_livox
)

### Generating Module File
_generate_module_eus(lio_livox
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lio_livox
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lio_livox_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lio_livox_generate_messages lio_livox_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_dependencies(lio_livox_generate_messages_eus _lio_livox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_dependencies(lio_livox_generate_messages_eus _lio_livox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lio_livox_geneus)
add_dependencies(lio_livox_geneus lio_livox_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lio_livox_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lio_livox
)

### Generating Services
_generate_srv_lisp(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lio_livox
)

### Generating Module File
_generate_module_lisp(lio_livox
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lio_livox
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lio_livox_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lio_livox_generate_messages lio_livox_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_dependencies(lio_livox_generate_messages_lisp _lio_livox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_dependencies(lio_livox_generate_messages_lisp _lio_livox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lio_livox_genlisp)
add_dependencies(lio_livox_genlisp lio_livox_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lio_livox_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lio_livox
)

### Generating Services
_generate_srv_nodejs(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lio_livox
)

### Generating Module File
_generate_module_nodejs(lio_livox
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lio_livox
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lio_livox_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lio_livox_generate_messages lio_livox_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_dependencies(lio_livox_generate_messages_nodejs _lio_livox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_dependencies(lio_livox_generate_messages_nodejs _lio_livox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lio_livox_gennodejs)
add_dependencies(lio_livox_gennodejs lio_livox_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lio_livox_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/melodic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox
)

### Generating Services
_generate_srv_py(lio_livox
  "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox
)

### Generating Module File
_generate_module_py(lio_livox
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lio_livox_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lio_livox_generate_messages lio_livox_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/srv/save_map.srv" NAME_WE)
add_dependencies(lio_livox_generate_messages_py _lio_livox_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ya/lio-livox-modify_ws/src/LIO-Livox-modified/msg/cloud_info.msg" NAME_WE)
add_dependencies(lio_livox_generate_messages_py _lio_livox_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lio_livox_genpy)
add_dependencies(lio_livox_genpy lio_livox_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lio_livox_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lio_livox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lio_livox
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(lio_livox_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lio_livox_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(lio_livox_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(lio_livox_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lio_livox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lio_livox
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(lio_livox_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lio_livox_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(lio_livox_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(lio_livox_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lio_livox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lio_livox
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(lio_livox_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lio_livox_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(lio_livox_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(lio_livox_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lio_livox)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lio_livox
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(lio_livox_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lio_livox_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(lio_livox_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(lio_livox_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lio_livox
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(lio_livox_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lio_livox_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(lio_livox_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(lio_livox_generate_messages_py sensor_msgs_generate_messages_py)
endif()
