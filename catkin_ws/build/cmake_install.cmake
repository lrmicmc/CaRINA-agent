# Install script for directory: /home/luis/carla/TRACK/team_code/catkin_ws/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/luis/carla/TRACK/team_code/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE PROGRAM FILES "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE PROGRAM FILES "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/setup.bash;/home/luis/carla/TRACK/team_code/catkin_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE FILE FILES
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/setup.bash"
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/setup.sh;/home/luis/carla/TRACK/team_code/catkin_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE FILE FILES
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/setup.sh"
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/setup.zsh;/home/luis/carla/TRACK/team_code/catkin_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE FILE FILES
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/setup.zsh"
    "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/luis/carla/TRACK/team_code/catkin_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/luis/carla/TRACK/team_code/catkin_ws/install" TYPE FILE FILES "/home/luis/carla/TRACK/team_code/catkin_ws/build/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/gtest/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/ros_numpy/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/ackermann_msgs/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/msgs_action/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/msgs_mapping/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/msgs_traffic/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/carina_2d_detection/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/carina_3d_detection/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/carina/carina_bridge/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/cnn_planner/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/control/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/dataset/create_dataset_imitation_learning/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/dataset/create_images_video/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/emergency_monitor/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/carina/fault_detection/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/fpn_planner/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/mpc_traj_generator/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/carina/status_publisher/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/utils/timed_roslaunch/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/msgs_perception/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/collision_detection/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/decision_making/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/messages/msgs_navigation/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/global_path_planning/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/global_plan_monitor/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/local_path_planning/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/obstacle_detection/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/carina/carina_description/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/LRM-State-Estimation/lidar_object_tracking/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/elas_lrm/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/navigation/LRM-Localization-Stack-2023/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/lidar_birds_eye_view/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/traffic_signs/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/carina/carina_master/cmake_install.cmake")
  include("/home/luis/carla/TRACK/team_code/catkin_ws/build/perception/polar_height_map/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/luis/carla/TRACK/team_code/catkin_ws/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
