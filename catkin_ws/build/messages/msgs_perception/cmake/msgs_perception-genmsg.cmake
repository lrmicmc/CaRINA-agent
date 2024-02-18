# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "msgs_perception: 5 messages, 0 services")

set(MSG_I_FLAGS "-Imsgs_perception:/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(msgs_perception_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_custom_target(_msgs_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_perception" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" "geometry_msgs/Point:geometry_msgs/Quaternion:sensor_msgs/PointField:std_msgs/Header:geometry_msgs/Vector3:std_msgs/String:std_msgs/ColorRGBA:geometry_msgs/Twist:sensor_msgs/PointCloud2:geometry_msgs/Pose:msgs_perception/BoundingBox"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_custom_target(_msgs_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_perception" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" "msgs_perception/Obstacle:geometry_msgs/Point:geometry_msgs/Quaternion:sensor_msgs/PointField:std_msgs/Header:geometry_msgs/Vector3:std_msgs/String:std_msgs/ColorRGBA:geometry_msgs/Twist:sensor_msgs/PointCloud2:geometry_msgs/Pose:msgs_perception/BoundingBox"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_custom_target(_msgs_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_perception" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" "geometry_msgs/Point:std_msgs/String"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_custom_target(_msgs_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_perception" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" "geometry_msgs/Point:std_msgs/String:std_msgs/Header:msgs_perception/BoundingBox"
)

get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_custom_target(_msgs_perception_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "msgs_perception" "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" "sensor_msgs/PointCloud2:sensor_msgs/PointField:std_msgs/Header:sensor_msgs/Image"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
)
_generate_msg_cpp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
)
_generate_msg_cpp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
)
_generate_msg_cpp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
)
_generate_msg_cpp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
)

### Generating Services

### Generating Module File
_generate_module_cpp(msgs_perception
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(msgs_perception_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(msgs_perception_generate_messages msgs_perception_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_cpp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_cpp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_cpp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_cpp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_cpp _msgs_perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_perception_gencpp)
add_dependencies(msgs_perception_gencpp msgs_perception_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_perception_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
)
_generate_msg_eus(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
)
_generate_msg_eus(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
)
_generate_msg_eus(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
)
_generate_msg_eus(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
)

### Generating Services

### Generating Module File
_generate_module_eus(msgs_perception
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(msgs_perception_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(msgs_perception_generate_messages msgs_perception_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_eus _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_eus _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_eus _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_eus _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_eus _msgs_perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_perception_geneus)
add_dependencies(msgs_perception_geneus msgs_perception_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_perception_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
)
_generate_msg_lisp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
)
_generate_msg_lisp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
)
_generate_msg_lisp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
)
_generate_msg_lisp(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
)

### Generating Services

### Generating Module File
_generate_module_lisp(msgs_perception
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(msgs_perception_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(msgs_perception_generate_messages msgs_perception_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_lisp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_lisp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_lisp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_lisp _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_lisp _msgs_perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_perception_genlisp)
add_dependencies(msgs_perception_genlisp msgs_perception_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_perception_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
)
_generate_msg_nodejs(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
)
_generate_msg_nodejs(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
)
_generate_msg_nodejs(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
)
_generate_msg_nodejs(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
)

### Generating Services

### Generating Module File
_generate_module_nodejs(msgs_perception
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(msgs_perception_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(msgs_perception_generate_messages msgs_perception_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_nodejs _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_nodejs _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_nodejs _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_nodejs _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_nodejs _msgs_perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_perception_gennodejs)
add_dependencies(msgs_perception_gennodejs msgs_perception_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_perception_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
)
_generate_msg_py(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg"
  "${MSG_I_FLAGS}"
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
)
_generate_msg_py(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
)
_generate_msg_py(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/String.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
)
_generate_msg_py(msgs_perception
  "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointCloud2.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/PointField.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
)

### Generating Services

### Generating Module File
_generate_module_py(msgs_perception
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(msgs_perception_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(msgs_perception_generate_messages msgs_perception_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/Obstacle.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_py _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/ObstacleArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_py _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBox.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_py _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/BoundingBoxArray.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_py _msgs_perception_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/luis/carla/TRACK/team_code/catkin_ws/src/messages/msgs_perception/msg/StereoCloudImage.msg" NAME_WE)
add_dependencies(msgs_perception_generate_messages_py _msgs_perception_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(msgs_perception_genpy)
add_dependencies(msgs_perception_genpy msgs_perception_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS msgs_perception_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/msgs_perception
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(msgs_perception_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(msgs_perception_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(msgs_perception_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/msgs_perception
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(msgs_perception_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(msgs_perception_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(msgs_perception_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/msgs_perception
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(msgs_perception_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(msgs_perception_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(msgs_perception_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/msgs_perception
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(msgs_perception_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(msgs_perception_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(msgs_perception_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/msgs_perception
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(msgs_perception_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(msgs_perception_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(msgs_perception_generate_messages_py sensor_msgs_generate_messages_py)
endif()
