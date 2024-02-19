# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msgs_navigation: 6 messages, 0 services")

set(MSG_I_FLAGS "-Imsgs_navigation:/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Imsgs_navigation:/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Imsgs_perception:/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msgs_navigation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" "msgs_navigation/TrajectoryPoint:std_msgs/Header"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" ""
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" "geometry_msgs/Point:geometry_msgs/Vector3:msgs_perception/Obstacle:msgs_perception/BoundingBox:std_msgs/ColorRGBA:geometry_msgs/Quaternion:std_msgs/Header:sensor_msgs/PointField:sensor_msgs/PointCloud2:geometry_msgs/Twist:std_msgs/String:geometry_msgs/Pose"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_custom_target(_msgs_navigation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_navigation" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_cpp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
)

### Generating Services

### Generating Module File
_generate_module_cpp(msgs_navigation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msgs_navigation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msgs_navigation_generate_messages msgs_navigation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_cpp _msgs_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_navigation_gencpp)
add_dependencies(msgs_navigation_gencpp msgs_navigation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_navigation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)
_generate_msg_eus(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
)

### Generating Services

### Generating Module File
_generate_module_eus(msgs_navigation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(msgs_navigation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(msgs_navigation_generate_messages msgs_navigation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_eus _msgs_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_navigation_geneus)
add_dependencies(msgs_navigation_geneus msgs_navigation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_navigation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)
_generate_msg_lisp(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
)

### Generating Services

### Generating Module File
_generate_module_lisp(msgs_navigation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msgs_navigation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msgs_navigation_generate_messages msgs_navigation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_lisp _msgs_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_navigation_genlisp)
add_dependencies(msgs_navigation_genlisp msgs_navigation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_navigation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)
_generate_msg_nodejs(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(msgs_navigation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(msgs_navigation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(msgs_navigation_generate_messages msgs_navigation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_nodejs _msgs_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_navigation_gennodejs)
add_dependencies(msgs_navigation_gennodejs msgs_navigation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_navigation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)
_generate_msg_py(msgs_navigation
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
)

### Generating Services

### Generating Module File
_generate_module_py(msgs_navigation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msgs_navigation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msgs_navigation_generate_messages msgs_navigation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/Path.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryPoint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/GlobalPlan.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/SpeedConstraint.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_navigation/msg/TrajectoryError.msg" NAME_WE)
add_dependencies(msgs_navigation_generate_messages_py _msgs_navigation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_navigation_genpy)
add_dependencies(msgs_navigation_genpy msgs_navigation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_navigation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_navigation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(msgs_navigation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET msgs_navigation_generate_messages_cpp)
  add_dependencies(msgs_navigation_generate_messages_cpp msgs_navigation_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(msgs_navigation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET msgs_perception_generate_messages_cpp)
  add_dependencies(msgs_navigation_generate_messages_cpp msgs_perception_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_navigation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(msgs_navigation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET msgs_navigation_generate_messages_eus)
  add_dependencies(msgs_navigation_generate_messages_eus msgs_navigation_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(msgs_navigation_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET msgs_perception_generate_messages_eus)
  add_dependencies(msgs_navigation_generate_messages_eus msgs_perception_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_navigation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(msgs_navigation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET msgs_navigation_generate_messages_lisp)
  add_dependencies(msgs_navigation_generate_messages_lisp msgs_navigation_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(msgs_navigation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET msgs_perception_generate_messages_lisp)
  add_dependencies(msgs_navigation_generate_messages_lisp msgs_perception_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_navigation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(msgs_navigation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET msgs_navigation_generate_messages_nodejs)
  add_dependencies(msgs_navigation_generate_messages_nodejs msgs_navigation_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(msgs_navigation_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET msgs_perception_generate_messages_nodejs)
  add_dependencies(msgs_navigation_generate_messages_nodejs msgs_perception_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_navigation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(msgs_navigation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET msgs_navigation_generate_messages_py)
  add_dependencies(msgs_navigation_generate_messages_py msgs_navigation_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(msgs_navigation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET msgs_perception_generate_messages_py)
  add_dependencies(msgs_navigation_generate_messages_py msgs_perception_generate_messages_py)
endif()
