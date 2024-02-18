execute_process(COMMAND "/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/ros_numpy/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/ros_numpy/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
